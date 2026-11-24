#include <cmath>
#include <deque>
#include <functional>
#include <optional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "imu_propagation/imu_propagator.hpp"




class ImuPropagationNode : public rclcpp::Node
{
public:
	ImuPropagationNode()
		: rclcpp::Node("imu_propagation")
	{
		RCLCPP_INFO(
			get_logger(),
			"IMU propagation node started.");

		auto imu_qos = rclcpp::SensorDataQoS();
		imu_qos.keep_last(2000);

		imu_subscription_ =
			create_subscription<sensor_msgs::msg::Imu>(
				"/imu0",
				imu_qos,
				std::bind(
					&ImuPropagationNode::imu_callback,
					this,
					std::placeholders::_1));
	}

private:
	void imu_callback(
		const sensor_msgs::msg::Imu::ConstSharedPtr &message)
    {
        const auto current_measurement =
            to_imu_measurement(*message);

        if (!is_valid_measurement(current_measurement)) {
            RCLCPP_WARN(
                get_logger(),
                "Received invalid IMU measurement.");
            return;
        }

        if (!initialized_) {
            process_initialization_measurement(
                current_measurement);
            return;
        }


        const auto &previous_measurement =
            previous_measurement_.value();

        const double dt =
            current_measurement.timestamp -
            previous_measurement.timestamp;

        if (dt <= 0.0) {
            RCLCPP_WARN(
                get_logger(),
                "IMU timestamp is not strictly increasing.");

            restart_initialization(current_measurement);
            return;
        }

        if (dt > 0.1) {
            RCLCPP_WARN(
                get_logger(),
                "Large IMU interval: %.6f s",
                dt);

            restart_initialization(current_measurement);
            return;
        }

        if (!propagator_.propagate(
                previous_measurement,
                current_measurement)) {
            RCLCPP_WARN(
                get_logger(),
                "Failed to propagate IMU state.");

            return;
        }
        previous_measurement_ = current_measurement;

        const auto &state = propagator_.state();

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "p=[%.3f, %.3f, %.3f], "
            "v=[%.3f, %.3f, %.3f], "
            "q_wxyz=[%.4f, %.4f, %.4f, %.4f]",
            state.position_wb.x(),
            state.position_wb.y(),
            state.position_wb.z(),
            state.velocity_wb.x(),
            state.velocity_wb.y(),
            state.velocity_wb.z(),
            state.orientation_wb.w(),
            state.orientation_wb.x(),
            state.orientation_wb.y(),
            state.orientation_wb.z());
    }


    static imu_propagation::ImuMeasurement to_imu_measurement(
        const sensor_msgs::msg::Imu &message)
    {
        imu_propagation::ImuMeasurement measurement;

        measurement.timestamp =
            rclcpp::Time(message.header.stamp).seconds();

        measurement.acceleration =
            Eigen::Vector3d(
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z);

        measurement.angular_velocity =
            Eigen::Vector3d(
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z);

        return measurement;
    }

    static bool is_valid_measurement(
        const imu_propagation::ImuMeasurement &measurement)
    {
        return std::isfinite(measurement.timestamp) &&
            measurement.acceleration.allFinite() &&
            measurement.angular_velocity.allFinite();
    }

    

    void process_initialization_measurement(
        const imu_propagation::ImuMeasurement &measurement)
    {
        if (!initialization_buffer_.empty()) {
            const double dt =
                measurement.timestamp -
                initialization_buffer_.back().timestamp;

            if (dt <= 0.0 || dt > 0.1) {
                RCLCPP_WARN(
                    get_logger(),
                    "IMU discontinuity during initialization.");

                initialization_buffer_.clear();
            }
        }

        initialization_buffer_.push_back(measurement);

        while (initialization_buffer_.size() >
            kInitializationSampleCount) {
            initialization_buffer_.pop_front();
        }

        if (initialization_buffer_.size() <
            kInitializationSampleCount) {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Collecting initialization IMU: %zu/%zu",
                initialization_buffer_.size(),
                kInitializationSampleCount);

            return;
        }

        try_initialize();
    }

    bool try_initialize()
    {
        Eigen::Vector3d mean_acceleration =
            Eigen::Vector3d::Zero();

        Eigen::Vector3d mean_angular_velocity =
            Eigen::Vector3d::Zero();

        for (const auto &measurement :
            initialization_buffer_) {
            mean_acceleration +=
                measurement.acceleration;

            mean_angular_velocity +=
                measurement.angular_velocity;
        }

        const double sample_count =
            static_cast<double>(
                initialization_buffer_.size());

        mean_acceleration /= sample_count;
        mean_angular_velocity /= sample_count;

        double acceleration_squared_error = 0.0;
        double gyroscope_squared_error = 0.0;

        for (const auto &measurement :
            initialization_buffer_) {
            acceleration_squared_error +=
                (measurement.acceleration -
                mean_acceleration).squaredNorm();

            gyroscope_squared_error +=
                (measurement.angular_velocity -
                mean_angular_velocity).squaredNorm();
        }

        const double acceleration_rms =
            std::sqrt(
                acceleration_squared_error /
                sample_count);

        const double gyroscope_rms =
            std::sqrt(
                gyroscope_squared_error /
                sample_count);

        const double acceleration_norm =
            mean_acceleration.norm();

        const bool is_stationary =
            std::abs(
                acceleration_norm -
                kGravityMagnitude) <
                kMaximumAccelerationDeviation &&
            mean_angular_velocity.norm() <
                kMaximumMeanAngularVelocity &&
            acceleration_rms <
                kMaximumAccelerometerRms &&
            gyroscope_rms <
                kMaximumGyroscopeRms;

        if (!is_stationary) {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Waiting for stationary IMU: "
                "|a|=%.3f, |w|=%.4f, "
                "acc_rms=%.3f, gyro_rms=%.4f",
                acceleration_norm,
                mean_angular_velocity.norm(),
                acceleration_rms,
                gyroscope_rms);

            return false;
        }

        const Eigen::Vector3d measured_up_body =
            mean_acceleration.normalized();

        const Eigen::Quaterniond initial_orientation =
            Eigen::Quaterniond::FromTwoVectors(
                measured_up_body,
                Eigen::Vector3d::UnitZ());

        imu_propagation::ImuState initial_state;

        initial_state.timestamp =
            initialization_buffer_.back().timestamp;

        initial_state.orientation_wb =
            initial_orientation;

        initial_state.gyroscope_bias =
            mean_angular_velocity;

        if (!propagator_.reset(initial_state)) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to initialize IMU propagator.");
            return false;
        }

        previous_measurement_ =
            initialization_buffer_.back();

        initialized_ = true;
        initialization_buffer_.clear();

        RCLCPP_INFO(
            get_logger(),
            "IMU initialized: "
            "|a|=%.3f, "
            "gyro_bias=[%.6f, %.6f, %.6f]",
            acceleration_norm,
            mean_angular_velocity.x(),
            mean_angular_velocity.y(),
            mean_angular_velocity.z());

        return true;
    }    

    void restart_initialization(
        const imu_propagation::ImuMeasurement &measurement)
    {
        initialized_ = false;
        previous_measurement_.reset();
        initialization_buffer_.clear();

        propagator_.reset();

        initialization_buffer_.push_back(
            measurement);
    }

	rclcpp::Subscription<
		sensor_msgs::msg::Imu>::SharedPtr
		imu_subscription_;

	imu_propagation::ImuPropagator propagator_;

	std::optional<
		imu_propagation::ImuMeasurement>
		previous_measurement_;

    std::deque<
        imu_propagation::ImuMeasurement>
        initialization_buffer_;

    bool initialized_ = false;

    static constexpr std::size_t
        kInitializationSampleCount = 200;

    static constexpr double
        kGravityMagnitude = 9.81;

    static constexpr double
        kMaximumMeanAngularVelocity = 0.1;

    static constexpr double
        kMaximumAccelerationDeviation = 0.5;

    static constexpr double
        kMaximumGyroscopeRms = 0.02;

    static constexpr double
        kMaximumAccelerometerRms = 0.3;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	const auto node =
		std::make_shared<ImuPropagationNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}