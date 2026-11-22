#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace vio_data_sync
{

double stampToSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

struct Measurement
{
  sensor_msgs::msg::Image::ConstSharedPtr image;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imus;
  double target_time{0.0};
};

class DataSyncNode final : public rclcpp::Node
{
public:
  DataSyncNode()
  : Node("vio_data_sync")
  {
    const auto image_topic = declare_parameter<std::string>("image_topic", "/cam0/image_raw");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu0");
    time_offset_ = declare_parameter<double>("time_offset", 0.0);

    auto image_qos = rclcpp::SensorDataQoS().keep_last(100);
    auto imu_qos = rclcpp::SensorDataQoS().keep_last(2000);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, image_qos,
      std::bind(&DataSyncNode::imageCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, imu_qos,
      std::bind(&DataSyncNode::imuCallback, this, std::placeholders::_1));

    worker_ = std::thread(&DataSyncNode::processLoop, this);

    RCLCPP_INFO(
      get_logger(), "waiting for image=%s and imu=%s, time_offset=%.6f s",
      image_topic.c_str(), imu_topic.c_str(), time_offset_);
  }

  ~DataSyncNode() override
  {
    running_.store(false);
    condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    const double stamp = stampToSeconds(msg->header.stamp);
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      if (stamp <= last_imu_time_) {
        RCLCPP_WARN(
          get_logger(), "discard out-of-order IMU: current=%.9f last=%.9f",
          stamp, last_imu_time_);
        return;
      }
      last_imu_time_ = stamp;
      imu_buffer_.push_back(std::move(msg));
    }
    condition_.notify_one();
  }

  void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    const double stamp = stampToSeconds(msg->header.stamp);
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      if (stamp <= last_image_time_) {
        RCLCPP_WARN(
          get_logger(), "discard out-of-order image: current=%.9f last=%.9f",
          stamp, last_image_time_);
        return;
      }
      last_image_time_ = stamp;
      image_buffer_.push_back(std::move(msg));
    }
    condition_.notify_one();
  }

  std::vector<Measurement> getMeasurementsLocked()
  {
    std::vector<Measurement> measurements;

    while (!imu_buffer_.empty() && !image_buffer_.empty()) {
      const double image_time = stampToSeconds(image_buffer_.front()->header.stamp);
      const double target_time = image_time + time_offset_;
      const double newest_imu_time = stampToSeconds(imu_buffer_.back()->header.stamp);

      if (newest_imu_time <= target_time) {
        break;
      }

      const double oldest_imu_time = stampToSeconds(imu_buffer_.front()->header.stamp);
      if (oldest_imu_time >= target_time) {
        RCLCPP_WARN(
          get_logger(),
          "drop image %.9f: oldest IMU %.9f does not precede target %.9f",
          image_time, oldest_imu_time, target_time);
        image_buffer_.pop_front();
        ++dropped_images_;
        continue;
      }

      Measurement measurement;
      measurement.image = image_buffer_.front();
      measurement.target_time = target_time;
      image_buffer_.pop_front();

      while (
        !imu_buffer_.empty() &&
        stampToSeconds(imu_buffer_.front()->header.stamp) < target_time)
      {
        measurement.imus.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
      }

      // Keep this first IMU at/after the image time in the global buffer. It is
      // the right interpolation endpoint now and the left-side sample next time.
      measurement.imus.push_back(imu_buffer_.front());
      measurements.push_back(std::move(measurement));
    }

    return measurements;
  }

  void processLoop()
  {
    while (running_.load()) {
      std::vector<Measurement> measurements;
      {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        condition_.wait(lock, [this, &measurements]() {
          if (!running_.load()) {
            return true;
          }
          measurements = getMeasurementsLocked();
          return !measurements.empty();
        });
        if (!running_.load()) {
          break;
        }
      }

      for (const auto & measurement : measurements) {
        const double image_time = stampToSeconds(measurement.image->header.stamp);
        const double first_imu_time = stampToSeconds(measurement.imus.front()->header.stamp);
        const double last_imu_time = stampToSeconds(measurement.imus.back()->header.stamp);
        ++synced_images_;

        RCLCPP_INFO(
          get_logger(),
          "sync #%zu image=%.9f target=%.9f imus=%zu range=[%.9f, %.9f] "
          "bracket=%s dropped=%zu",
          synced_images_, image_time, measurement.target_time, measurement.imus.size(),
          first_imu_time, last_imu_time,
          first_imu_time < measurement.target_time &&
          last_imu_time >= measurement.target_time ? "yes" : "NO",
          dropped_images_);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> image_buffer_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
  std::mutex buffer_mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  std::atomic<bool> running_{true};

  double time_offset_{0.0};
  double last_imu_time_{-1.0};
  double last_image_time_{-1.0};
  std::size_t synced_images_{0};
  std::size_t dropped_images_{0};
};

}  // namespace vio_data_sync

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vio_data_sync::DataSyncNode>());
  rclcpp::shutdown();
  return 0;
}
