#include <chrono>
#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <functional>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>


class FeatureTrackerNode : public rclcpp::Node
{

public:
	FeatureTrackerNode()
        : rclcpp::Node("feature_tracker")
    {
        RCLCPP_INFO(get_logger(), "Feature tracker node started.");

        auto image_qos = rclcpp::SensorDataQoS();
        image_qos.keep_last(10);

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/cam0/image_raw",
            image_qos,
            std::bind(&FeatureTrackerNode::image_callback, this, std::placeholders::_1)
        );

		visualization_publisher_ = create_publisher<sensor_msgs::msg::Image>(
			"feature_tracker/visualization",
			10);
		

    }

	~FeatureTrackerNode() override
	{
		RCLCPP_INFO(
			get_logger(),
			"Received %zu images",
			received_image_count_
		);
	}

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &message)
    {
		try{
			const auto cv_image = cv_bridge::toCvShare(
				message,
				sensor_msgs::image_encodings::MONO8);

        	const cv::Mat& gray_image = cv_image->image;

			++received_image_count_;
			const std::int64_t current_timestamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
			if (previous_timestamp_ns_ >= 0) {
				const double interval = static_cast<double>(current_timestamp_ns - previous_timestamp_ns_) / 1.0e9;
				if (interval > 0.06) {
					RCLCPP_WARN(
						get_logger(), "Large image interval: %.6f s", interval);
				}
			}
			previous_timestamp_ns_ = current_timestamp_ns;

			
			const auto start_time = std::chrono::steady_clock::now();
			const auto feature_points = detect_features(gray_image);
			
			const cv::Mat visualization_image = draw_features(gray_image, feature_points);
			auto visualization_message = cv_bridge::CvImage(
				message->header,
				sensor_msgs::image_encodings::BGR8,
				visualization_image).toImageMsg();

			visualization_publisher_->publish(*visualization_message);
			
			const auto end_time = std::chrono::steady_clock::now();
			const double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

			RCLCPP_INFO_THROTTLE(
				get_logger(),
				*get_clock(),
				1000,
				"Processed %zu points in %.3f ms",
				feature_points.size(),
				elapsed_ms);
			


		} catch (const cv_bridge::Exception &exception) {
			RCLCPP_ERROR(
				get_logger(),
				"Failed to convert image: %s",
				exception.what()
			);
		}

		
    }   

	std::vector<cv::Point2f> detect_features(
		const cv::Mat &gray_image) const
	{
		std::vector<cv::Point2f> feature_points;
		cv::goodFeaturesToTrack(gray_image, feature_points, kMaxFeatureCount, kQualityLevel, kMinFeatureDistance);

		return feature_points;
	}

	cv::Mat draw_features(const cv::Mat &gray_image, const std::vector<cv::Point2f> &feature_points) const
	{
		cv::Mat visualization_image;
		cv::cvtColor(gray_image, visualization_image, cv::COLOR_GRAY2BGR);
		for (const auto &point : feature_points) {
			cv::circle(visualization_image, point, 3, cv::Scalar(0, 0, 255), -1);
		}

		return visualization_image;
	}

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

	static constexpr int kMaxFeatureCount = 150;
	static constexpr double kQualityLevel = 0.01;
	static constexpr double kMinFeatureDistance = 30.0;

	std::size_t received_image_count_ = 0;
	std::int64_t previous_timestamp_ns_ = -1;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_publisher_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}