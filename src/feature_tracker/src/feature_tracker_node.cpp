#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <functional>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
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
		try {
			const auto cv_image = cv_bridge::toCvShare(
				message,
				sensor_msgs::image_encodings::MONO8);
			
        	const cv::Mat &gray_image = cv_image->image;

			++received_image_count_;
			const std::int64_t current_timestamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
			if (previous_timestamp_ns_ >= 0) {
				const double interval = static_cast<double>(current_timestamp_ns - previous_timestamp_ns_) / 1.0e9;
				if (interval > 0.06) {
					RCLCPP_WARN(
						get_logger(), "Large image interval: %.6f s", interval);
				}
			}
			if (previous_timestamp_ns_ >= 0 && current_timestamp_ns <= previous_timestamp_ns_) {
				RCLCPP_WARN(get_logger(), "Image timestamp is not strictly increasing");

				previous_image_.release();
				previous_points_.clear();
				feature_ids_.clear();
				track_counts_.clear();
			}
			previous_timestamp_ns_ = current_timestamp_ns;

			
			const auto start_time = std::chrono::steady_clock::now();

			if (previous_image_.empty()) {
				initialize_features(gray_image);
				previous_image_ = gray_image.clone();

				const cv::Mat visualization_image = draw_features(gray_image, previous_points_);
				auto visualization_message = cv_bridge::CvImage(
					message->header,
					sensor_msgs::image_encodings::BGR8,
					visualization_image).toImageMsg();

				visualization_publisher_->publish(*visualization_message);
				return;
			}

			std::vector<cv::Point2f> current_points;
			std::vector<unsigned char> status;
			std::vector<float> errors;

			assert(previous_points_.size() == feature_ids_.size());
			assert(previous_points_.size() == track_counts_.size());

			cv::calcOpticalFlowPyrLK(
				previous_image_,
				gray_image,
				previous_points_,
				current_points,
				status,
				errors
			);

			std::vector<cv::Point2f> valid_current_points;
			std::vector<cv::Point2f> valid_previous_points;
			std::vector<int> valid_feature_ids;
			std::vector<int> valid_track_counts;

			for (std::size_t i = 0; i < current_points.size(); ++i) {
				if (status[i] && is_inside_image(current_points[i], gray_image)) {
					valid_current_points.push_back(current_points[i]);
					valid_previous_points.push_back(previous_points_[i]);

					valid_feature_ids.push_back(feature_ids_[i]);
					valid_track_counts.push_back(track_counts_[i] + 1);
				}
			}

			assert(valid_previous_points.size() == valid_current_points.size());
			assert(valid_current_points.size() == valid_feature_ids.size());
			assert(valid_current_points.size() == valid_track_counts.size());

			const std::size_t tracked_count = valid_current_points.size();
			cv::Mat visualization_image = draw_tracks(gray_image, valid_previous_points, valid_current_points, valid_track_counts);

			const cv::Mat feature_mask = create_feature_mask(gray_image, valid_current_points);

			const int required_feature_count = std::max(0, kMaxFeatureCount - static_cast<int>(valid_current_points.size()));
			const std::vector<cv::Point2f> new_points = detect_features(gray_image, required_feature_count, feature_mask);

			for (const auto &point : new_points) {
				valid_current_points.push_back(point);
				valid_feature_ids.push_back(next_feature_id_++);
				valid_track_counts.push_back(1);
				cv::circle(visualization_image, point, 3, cv::Scalar(255, 0, 0), -1);
			}

			assert(valid_current_points.size() == valid_feature_ids.size());
			assert(valid_current_points.size() == valid_track_counts.size());

			previous_image_ = gray_image.clone();
			previous_points_ = valid_current_points;
			feature_ids_ = valid_feature_ids;
			track_counts_ = valid_track_counts;

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
				"Tracked=%zu, added=%zu, total=%zu, time=%.3fms",
				tracked_count,
				new_points.size(),
				valid_current_points.size(),
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
		const cv::Mat &gray_image, int max_feature_count, const cv::Mat &mask = cv::Mat()) const
	{
		std::vector<cv::Point2f> feature_points;

		if (max_feature_count <= 0) {
			return feature_points;
		}
		cv::goodFeaturesToTrack(gray_image, feature_points, max_feature_count, kQualityLevel, kMinFeatureDistance, mask);

		return feature_points;
	}

	cv::Mat draw_features(const cv::Mat &gray_image, const std::vector<cv::Point2f> &feature_points) const
	{
		cv::Mat visualization_image;
		cv::cvtColor(gray_image, visualization_image, cv::COLOR_GRAY2BGR);

		for (const auto &point : feature_points) {
			cv::circle(visualization_image, point, 3, cv::Scalar(255, 0, 0), -1);
		}

		return visualization_image;
	}

	cv::Mat draw_tracks(const cv::Mat &gray_image, const std::vector<cv::Point2f> &previous_points, 
		const std::vector<cv::Point2f> &current_points, const std::vector<int> &track_counts) const
	{
		assert(previous_points.size() == current_points.size());
		assert(current_points.size() == track_counts.size());

		cv::Mat visualization_image;
		cv::cvtColor(gray_image, visualization_image, cv::COLOR_GRAY2BGR);
		for (std::size_t i = 0; i < current_points.size(); ++i) {
			const float ratio = std::min(1.0F, static_cast<float>(track_counts[i]) / 20.0F);
			const cv::Scalar color(255.0 * (1.0F - ratio), 0.0, 255.0 * ratio);
			cv::line(visualization_image, previous_points[i], current_points[i], cv::Scalar(0, 255, 0), 1);
			cv::circle(visualization_image, current_points[i], 3, color, -1);
		}

		return visualization_image;
	}

	bool is_inside_image(const cv::Point2f &point, const cv::Mat &image) const
	{
		return point.x >= 1.0F &&
			point.y >= 1.0F &&
			point.x < static_cast<float>(image.cols - 1) &&
			point.y < static_cast<float>(image.rows - 1);
	}

	void initialize_features(const cv::Mat &gray_image)
	{
		previous_points_ = detect_features(gray_image, kMaxFeatureCount);

		feature_ids_.clear();
		track_counts_.clear();

		for (std::size_t i = 0; i < previous_points_.size(); ++i) {
			feature_ids_.push_back(next_feature_id_++);
			track_counts_.push_back(1);
		}

	}

	cv::Mat create_feature_mask(const cv::Mat &gray_image, const std::vector<cv::Point2f> &existing_points) const
	{
		cv::Mat mask(gray_image.rows, gray_image.cols, CV_8UC1, cv::Scalar(255));

		for (const auto &point : existing_points) {
			cv::circle(mask, point, static_cast<int>(kMinFeatureDistance), cv::Scalar(0), -1);
		}

		return mask;
	}


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

	static constexpr int kMaxFeatureCount = 150;
	static constexpr double kQualityLevel = 0.01;
	static constexpr double kMinFeatureDistance = 30.0;

	std::size_t received_image_count_ = 0;
	std::int64_t previous_timestamp_ns_ = -1;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_publisher_;

	cv::Mat previous_image_;
	std::vector<cv::Point2f> previous_points_;

	std::vector<int> feature_ids_;
	std::vector<int> track_counts_;

	int next_feature_id_ = 0;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}