#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

struct Measurement
{
	sensor_msgs::msg::Image::ConstSharedPtr image;
	std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imus;
};

class DataSyncNode : public rclcpp::Node
{
public:
	DataSyncNode() 
		: rclcpp::Node("vio_data_sync")
	{
		RCLCPP_INFO(this->get_logger(), "VIO data synchronization node started");

		auto imu_qos = rclcpp::SensorDataQoS();
		imu_qos.keep_last(2000);
		auto image_qos = rclcpp::SensorDataQoS();
		image_qos.keep_last(100);

		imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
			"/imu0",
			imu_qos, 
			std::bind(&DataSyncNode::imu_callback, this, std::placeholders::_1)
		);

		image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
			"/cam0/image_raw",
			image_qos,
			std::bind(&DataSyncNode::image_callback, this, std::placeholders::_1)
		);

		process_thread_ = std::thread(&DataSyncNode::process_loop, this);
	}

	~DataSyncNode() override
	{
		{
			std::lock_guard<std::mutex> lock(buffer_mutex_);
			running_ = false;
		}

		data_condition_.notify_all();

		if (process_thread_.joinable()) {
			process_thread_.join();
		}
	}

private:
	void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
	{
		const auto current_timestamp_ns = get_timestamp_ns(msg->header.stamp);
		if (last_imu_timestamp_ns_ != -1 && current_timestamp_ns <= last_imu_timestamp_ns_) {
			RCLCPP_WARN_STREAM(get_logger(), "IMU timestamp is not strictly increasing: current="
				<< current_timestamp_ns
				<< ", last="
				<< last_imu_timestamp_ns_
			);
			return;
		}
		last_imu_timestamp_ns_ = current_timestamp_ns;
		
		{
			std::lock_guard<std::mutex> lock(buffer_mutex_);
			imu_buffer_.push_back(msg);
		}
		data_condition_.notify_one();
	}

	void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
	{
		const auto current_timestamp_ns = get_timestamp_ns(msg->header.stamp);
		if (last_image_timestamp_ns_ != -1 && current_timestamp_ns <= last_image_timestamp_ns_) {
			RCLCPP_WARN_STREAM(get_logger(), "Image timestamp is not strictly increasing: current="
				<< current_timestamp_ns
				<< ", last="
				<< last_image_timestamp_ns_
			);
			return;
		}
		last_image_timestamp_ns_ = current_timestamp_ns;

		{
			std::lock_guard<std::mutex> lock(buffer_mutex_);
			image_buffer_.push_back(msg);
		}
		data_condition_.notify_one();

	}

	static std::int64_t get_timestamp_ns(const builtin_interfaces::msg::Time& stamp)
	{
		return rclcpp::Time(stamp).nanoseconds();
	}


	bool has_processable_data_locked() const
	{
		if (image_buffer_.empty() || imu_buffer_.empty())
			return false;
		const auto target_time = get_timestamp_ns(image_buffer_.front()->header.stamp);
		const auto oldest_imu_time = get_timestamp_ns(imu_buffer_.front()->header.stamp);
		const auto newest_imu_time = get_timestamp_ns(imu_buffer_.back()->header.stamp);

		return newest_imu_time > target_time || oldest_imu_time >= target_time;
	}

	std::vector<Measurement> get_measurements_locked()
	{
		std::vector<Measurement> measurements;

		while (true) {
			if (image_buffer_.empty() || imu_buffer_.empty())
				break;

			const auto image_time = get_timestamp_ns(image_buffer_.front()->header.stamp);
			const auto target_time = image_time;

			const auto newest_imu_time = get_timestamp_ns(imu_buffer_.back()->header.stamp);
			if (newest_imu_time <= target_time) {
				break;
			}

			const auto oldest_imu_time = get_timestamp_ns(imu_buffer_.front()->header.stamp);
			if (oldest_imu_time >= target_time) {
				RCLCPP_WARN_STREAM(
					get_logger(),
					"Dropping image because no earlier IMU is available:"
						<< " image=" << image_time
						<< ", oldest_imu=" << oldest_imu_time
				);
				image_buffer_.pop_front();
				continue;
			}
			
			Measurement measurement;
			measurement.image = image_buffer_.front();
			image_buffer_.pop_front();

			while (get_timestamp_ns(imu_buffer_.front()->header.stamp) < target_time) {
				measurement.imus.push_back(imu_buffer_.front());
				imu_buffer_.pop_front();
			}
			measurement.imus.push_back(imu_buffer_.front());
			measurements.push_back(std::move(measurement));
		}

		return measurements;
	}

	void log_measurements(const std::vector<Measurement>& measurements)
	{
		for (const auto& measurement : measurements) {
			const auto image_time = get_timestamp_ns(measurement.image->header.stamp);
			if (measurement.imus.empty()) {
				RCLCPP_ERROR(get_logger(), "Synchronized measurement contains no IMU data");
				continue;
			}
			const auto imu_count = measurement.imus.size();
			const auto first_imu_time = get_timestamp_ns(measurement.imus.front()->header.stamp);
			const auto last_imu_time = get_timestamp_ns(measurement.imus.back()->header.stamp);

			const auto target_time = image_time;

			const bool bracketed = first_imu_time < target_time && last_imu_time >= target_time;
			if (!bracketed) {
				RCLCPP_ERROR_STREAM(
					get_logger(),
					"Invalid synchronized measurement:"
						<< " image=" << image_time
						<< ", imu_range=[" << first_imu_time
						<< ", " << last_imu_time << "]"
				);
				continue;
			}
			RCLCPP_INFO_STREAM(
				get_logger(),
				"Synchronized measurement:"
					<< " image=" << image_time
					<< ", imu_count=" << imu_count
					<< ", imu_range=[" << first_imu_time
					<< ", " << last_imu_time << "]"
					<< ", bracketed="
					<< (bracketed ? "true" : "false")
			);
		}

	}

	void process_loop()
	{
		while (true) {
			std::vector<Measurement> measurements;
			{
				std::unique_lock<std::mutex> lock(buffer_mutex_);

				data_condition_.wait(
					lock,
					[this]() {
						return !running_ || has_processable_data_locked();
					}
				);

				if (!running_) {
					break;
				}
				measurements = get_measurements_locked();
			}
			log_measurements(measurements);
		}
	}

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
	std::int64_t last_imu_timestamp_ns_{-1};

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
	std::int64_t last_image_timestamp_ns_{-1};

	std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
	std::deque<sensor_msgs::msg::Image::ConstSharedPtr> image_buffer_;
	std::mutex buffer_mutex_;

	std::condition_variable data_condition_;
	std::thread process_thread_;
	bool running_{true};
};


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<DataSyncNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}

