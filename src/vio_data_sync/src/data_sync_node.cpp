#include <cstdint>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"



class DataSyncNode : public rclcpp::Node
{
public:
	DataSyncNode() 
		: rclcpp::Node("vio_data_sync")
	{
		RCLCPP_INFO(this->get_logger(), "VIO data synchronization node started");
		imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
			"/imu0",
			rclcpp::SensorDataQoS(), 
			std::bind(&DataSyncNode::imu_callback, this, std::placeholders::_1)
		);

		image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
			"/cam0/image_raw",
			rclcpp::SensorDataQoS(),
			std::bind(&DataSyncNode::image_callback, this, std::placeholders::_1)
		);
	}

private:
	void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
	{
		const auto current_timestamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
		if (last_imu_timestamp_ns_ != -1 && current_timestamp_ns <= last_imu_timestamp_ns_) {
			RCLCPP_WARN_STREAM(get_logger(), "IMU timestamp is not strictly increasing: current="
				<< current_timestamp_ns
				<< ", last="
				<< last_imu_timestamp_ns_
			);
			return;
		}
		last_imu_timestamp_ns_ = current_timestamp_ns;
		RCLCPP_DEBUG(get_logger(), "IMU stamp: %d.%09u", msg->header.stamp.sec, msg->header.stamp.nanosec);
	}

	void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
	{
		const auto current_timestamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
		if (last_image_timestamp_ns_ != -1 && current_timestamp_ns <= last_image_timestamp_ns_) {
			RCLCPP_WARN_STREAM(get_logger(), "Image timestamp is not strictly increasing: current="
				<< current_timestamp_ns
				<< ", last="
				<< last_image_timestamp_ns_
			);
			return;
		}
		last_image_timestamp_ns_ = current_timestamp_ns;
		RCLCPP_INFO(get_logger(), "Image stamp: %d.%09u", msg->header.stamp.sec, msg->header.stamp.nanosec);
		RCLCPP_INFO_STREAM(get_logger(), "Image width=" << msg->width
			<< ", height=" << msg->height
			<< ", encoding=" << msg->encoding	
		);

	}

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
	std::int64_t last_imu_timestamp_ns_{-1};

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
	std::int64_t last_image_timestamp_ns_{-1};
};


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<DataSyncNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}