#include "rclcpp/rclcpp.hpp"

#include <memory>

class DataSyncNode : public rclcpp::Node
{
public:
	DataSyncNode() 
		: rclcpp::Node("vio_data_sync")
	{
		RCLCPP_INFO(this->get_logger(), "VIO data synchronization node started");
	}


};


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<DataSyncNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}