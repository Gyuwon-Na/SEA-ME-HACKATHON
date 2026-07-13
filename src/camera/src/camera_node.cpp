#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <yaml-cpp/yaml.h>

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node") {
    config_file_ = declare_parameter<std::string>(
      "vehicle_config_file", "/home/topst/D-Racer-Kit/src/config/vehicle_config.yaml");
    topic_ = declare_parameter<std::string>("publish_topic", "/camera/image/compressed");
    publish_hz_ = declare_parameter<double>("publish_hz", 20.0);
    capture_hz_ = declare_parameter<double>("capture_hz", 20.0);
    usb_device_ = declare_parameter<std::string>("usb_camera_device", "/dev/video1");
    mipi_device_ = declare_parameter<std::string>("mipi_camera_device", "/dev/video0");
    flip_method_ = declare_parameter<std::string>("flip_method", "rotate-180");
    jpeg_quality_ = std::clamp(
      static_cast<int>(declare_parameter<int>("jpeg_quality", 90)), 0, 100);
    mjpg_passthrough_ = declare_parameter<bool>("mjpg_passthrough", true);
    require_mjpg_passthrough_ = declare_parameter<bool>("require_mjpg_passthrough", false);

    load_vehicle_config();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(topic_, qos);
    if (!open_capture()) {
      throw std::runtime_error("failed to open configured camera");
    }
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(publish_hz_, 1.0)),
      std::bind(&CameraNode::capture, this));
    report_started_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(), "C++ camera opened: device=%s %dx%d capture=%.1f publish=%.1f passthrough=%s",
      device_.c_str(), width_, height_, capture_hz_, publish_hz_, passthrough_ ? "true" : "false");
  }

  ~CameraNode() override { capture_.release(); }

private:
  void load_vehicle_config() {
    bool usb = true;
    try {
      const auto root = YAML::LoadFile(config_file_);
      if (root["USB_CAM"]) usb = root["USB_CAM"].as<bool>();
      if (root["IMAGE_WIDTH"]) width_ = root["IMAGE_WIDTH"].as<int>();
      if (root["IMAGE_HEIGHT"]) height_ = root["IMAGE_HEIGHT"].as<int>();
      if (root["USB_CAM_DEVICE"]) usb_device_ = root["USB_CAM_DEVICE"].as<std::string>();
      if (root["MIPI_CAM_DEVICE"]) mipi_device_ = root["MIPI_CAM_DEVICE"].as<std::string>();
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "vehicle config fallback: %s", error.what());
    }
    usb_camera_ = usb;
    device_ = usb_camera_ ? usb_device_ : mipi_device_;
  }

  bool open_mjpg_passthrough() {
    if (!capture_.open(device_, cv::CAP_V4L2)) return false;
    capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    capture_.set(cv::CAP_PROP_FPS, capture_hz_);
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1.0);
    capture_.set(cv::CAP_PROP_CONVERT_RGB, 0.0);
    for (int attempt = 0; attempt < 5; ++attempt) {
      cv::Mat raw;
      if (!capture_.read(raw) || raw.empty() || !raw.isContinuous()) continue;
      const auto * data = raw.ptr<std::uint8_t>();
      if (raw.total() < 2 || data[0] != 0xff || data[1] != 0xd8) break;
      const cv::Mat decoded = cv::imdecode(raw.reshape(1, 1), cv::IMREAD_COLOR);
      if (!decoded.empty() && decoded.cols == width_ && decoded.rows == height_) {
        passthrough_ = true;
        return true;
      }
      break;
    }
    capture_.release();
    return false;
  }

  bool open_capture() {
    if (usb_camera_ && mjpg_passthrough_ && open_mjpg_passthrough()) return true;
    if (usb_camera_ && require_mjpg_passthrough_) {
      RCLCPP_ERROR(
        get_logger(), "native MJPEG passthrough is required but unavailable on %s",
        device_.c_str());
      return false;
    }
    passthrough_ = false;
    const int fps = std::max(1, static_cast<int>(std::lround(capture_hz_)));
    std::string pipeline;
    if (usb_camera_) {
      pipeline = "v4l2src device=" + device_ + " io-mode=2 ! image/jpeg,framerate=" +
        std::to_string(fps) + "/1 ! jpegdec ! videoconvert ! videoscale ! "
        "video/x-raw,format=BGR,width=" + std::to_string(width_) + ",height=" +
        std::to_string(height_) + ",framerate=" + std::to_string(fps) +
        "/1 ! appsink sync=false drop=true max-buffers=1";
    } else {
      pipeline = "v4l2src device=" + device_ + " io-mode=2 ! video/x-raw,format=NV12,width=" +
        std::to_string(width_) + ",height=" + std::to_string(height_) + ",framerate=" +
        std::to_string(fps) + "/1 ! videoconvert ! videoflip method=" + flip_method_ +
        " ! video/x-raw,format=BGR ! appsink sync=false drop=true max-buffers=1";
    }
    return capture_.open(pipeline, cv::CAP_GSTREAMER);
  }

  void capture() {
    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "camera read failed");
      return;
    }
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera";
    msg.format = "jpeg";
    if (passthrough_) {
      if (!frame.isContinuous() || frame.total() < 2U) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "invalid MJPEG buffer");
        return;
      }
      const auto * begin = frame.ptr<std::uint8_t>();
      if (begin[0] != 0xff || begin[1] != 0xd8) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MJPEG frame has no JPEG SOI");
        return;
      }
      msg.data.assign(begin, begin + frame.total() * frame.elemSize());
    } else if (!cv::imencode(
        ".jpg", frame, msg.data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_})) {
      return;
    }
    publisher_->publish(msg);
    ++publish_count_;
    published_bytes_ += msg.data.size();
    const auto report_now = std::chrono::steady_clock::now();
    const double report_sec = std::chrono::duration<double>(report_now - report_started_).count();
    if (report_sec >= 3.0) {
      RCLCPP_INFO(
        get_logger(), "camera transfer: %.1f Hz, %.1f KiB/frame, passthrough=%s",
        publish_count_ / report_sec,
        published_bytes_ / static_cast<double>(std::max<uint64_t>(publish_count_, 1U)) / 1024.0,
        passthrough_ ? "true" : "false");
      publish_count_ = 0;
      published_bytes_ = 0;
      report_started_ = report_now;
    }
  }

  std::string config_file_, topic_, usb_device_, mipi_device_, device_, flip_method_;
  int width_{640}, height_{480}, jpeg_quality_{90};
  double publish_hz_{20.0}, capture_hz_{20.0};
  bool usb_camera_{true}, mjpg_passthrough_{true}, require_mjpg_passthrough_{false}, passthrough_{false};
  uint64_t publish_count_{0}, published_bytes_{0};
  std::chrono::steady_clock::time_point report_started_;
  cv::VideoCapture capture_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
