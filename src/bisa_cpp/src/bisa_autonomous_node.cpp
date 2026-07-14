#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/msg/control.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace {

double clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

struct LaneObs {
  bool valid{false};
  double center_error{0.0};
  double curvature{0.0};
  double signed_curvature{0.0};
  bool fork_seen{false};
  std::optional<double> left_target;
  std::optional<double> right_target;
};

struct HoughDebug {
  std::vector<cv::Vec4i> segments;
  std::optional<cv::Vec4d> left_line;
  std::optional<cv::Vec4d> right_line;
  cv::Mat edges;
  int top_y{0};
};

struct LaneDebug {
  cv::Rect roi;
  cv::Mat mask;
  HoughDebug hough;
  std::array<cv::Range, 3> bands;
  std::array<std::optional<double>, 3> centers;
};

struct Detection {
  int class_id{0};
  float confidence{0.0F};
  cv::Rect2f box;
};

struct StampedFrame {
  int32_t sec{0};
  uint32_t nanosec{0};
  cv::Mat image;
};

struct Command {
  double throttle{0.0};
  double steering{0.0};
};

struct Config {
  int lane_width{640};
  int lane_height{240};
  int lane_x{-1};
  int lane_y{-1};
  int lab_l_min{20}, lab_l_max{205};
  int lab_a_min{112}, lab_a_max{145};
  int lab_b_min{122}, lab_b_max{148};
  int white_l_min{200}, white_l_max{255};
  int white_a_min{115}, white_a_max{140};
  int white_b_min{115}, white_b_max{145};
  int yellow_l_min{80}, yellow_l_max{255};
  int yellow_a_min{100}, yellow_a_max{140};
  int yellow_b_min{165}, yellow_b_max{255};
  double clahe_clip{2.0};
  int clahe_tile{8};
  int morph_open{3}, morph_close{5};
  double min_area_ratio{0.006};
  double fork_area_ratio{0.035};
  double near_y0{0.70}, mid_y0{0.52}, mid_y1{0.75}, far_y0{0.32}, far_y1{0.58};
  double hough_top{0.45};
  int canny_low{50}, canny_high{150}, hough_threshold{38};
  int hough_min_length{30}, hough_max_gap{290};
  double hough_slope_min{0.22}, assumed_lane_width{0.62}, max_center_jump{0.70};
  double speed_min{0.20}, speed_max{0.22};
  double launch_cap{0.32}, s_curve_cap{0.24}, fork_approach_cap{0.20};
  double fork_commit_cap{0.25}, post_fork_cap{0.30}, post_fork_min{0.25};
  double ramp_up{0.015}, steer_slowdown{0.22}, curvature_slowdown{0.08};
  int steer_sign{1};
  double lookahead{0.60}, wheelbase{0.16}, lateral_scale{0.30};
  double max_steer_deg{30.0}, pp_gain{1.0}, curve_blend{1.0};
  double steer_rate{0.10}, straight_limit{0.45}, s_curve_limit{0.80};
  double fork_approach_limit{0.55}, fork_limit{0.85}, post_fork_limit{0.65};
  double lost_decay{0.70};
  bool color_enabled{true};
  double color_clahe_clip{2.0};
  int color_clahe_tile{8};
  double saturation_boost{1.5};
  int brightness{0};
  double contrast{1.0}, saturation{1.0}, gamma{1.0};
  int sign_vote_k{6}, sign_vote_n{10}, light_confirm_frames{8};
  double light_stale_sec{0.75};
  double launch_min_sec{1.0}, fork_commit_min_sec{0.8};
  double fork_commit_timeout_sec{1.8}, finish_min_elapsed_sec{8.0};
  int aruco_target_id{3};
};

template<typename T>
void read_value(const YAML::Node & node, const char * key, T & value) {
  if (node && node[key]) {
    value = node[key].as<T>();
  }
}

Config load_config(const std::string & path) {
  Config c;
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception &) {
    return c;
  }
  const auto roi = root["roi"];
  read_value(roi, "near_y0", c.near_y0); read_value(roi, "mid_y0", c.mid_y0);
  read_value(roi, "mid_y1", c.mid_y1); read_value(roi, "far_y0", c.far_y0);
  read_value(roi, "far_y1", c.far_y1);
  const auto lane_roi = root["lane_roi"];
  read_value(lane_roi, "width", c.lane_width); read_value(lane_roi, "height", c.lane_height);
  read_value(lane_roi, "x_offset", c.lane_x); read_value(lane_roi, "y_offset", c.lane_y);
  const auto lane = root["lane"];
  read_value(lane, "lab_l_min", c.lab_l_min); read_value(lane, "lab_l_max", c.lab_l_max);
  read_value(lane, "lab_a_min", c.lab_a_min); read_value(lane, "lab_a_max", c.lab_a_max);
  read_value(lane, "lab_b_min", c.lab_b_min); read_value(lane, "lab_b_max", c.lab_b_max);
  read_value(lane, "white_l_min", c.white_l_min); read_value(lane, "white_l_max", c.white_l_max);
  read_value(lane, "white_a_min", c.white_a_min); read_value(lane, "white_a_max", c.white_a_max);
  read_value(lane, "white_b_min", c.white_b_min); read_value(lane, "white_b_max", c.white_b_max);
  read_value(lane, "yellow_l_min", c.yellow_l_min); read_value(lane, "yellow_l_max", c.yellow_l_max);
  read_value(lane, "yellow_a_min", c.yellow_a_min); read_value(lane, "yellow_a_max", c.yellow_a_max);
  read_value(lane, "yellow_b_min", c.yellow_b_min); read_value(lane, "yellow_b_max", c.yellow_b_max);
  read_value(lane, "lab_clahe_clip", c.clahe_clip); read_value(lane, "lab_clahe_tile", c.clahe_tile);
  read_value(lane, "morph_open_kernel", c.morph_open); read_value(lane, "morph_close_kernel", c.morph_close);
  read_value(lane, "min_component_area_ratio", c.min_area_ratio);
  read_value(lane, "fork_area_ratio", c.fork_area_ratio);
  read_value(lane, "hough_roi_top_ratio", c.hough_top);
  read_value(lane, "hough_canny_low", c.canny_low); read_value(lane, "hough_canny_high", c.canny_high);
  read_value(lane, "hough_threshold", c.hough_threshold);
  read_value(lane, "hough_min_line_length", c.hough_min_length);
  read_value(lane, "hough_max_line_gap", c.hough_max_gap);
  read_value(lane, "hough_slope_min_abs", c.hough_slope_min);
  read_value(lane, "assumed_lane_width_ratio", c.assumed_lane_width);
  read_value(lane, "max_center_jump", c.max_center_jump);
  const auto detector = root["detector"];
  read_value(detector, "sign_vote_k", c.sign_vote_k); read_value(detector, "sign_vote_n", c.sign_vote_n);
  read_value(detector, "light_confirm_frames", c.light_confirm_frames);
  read_value(detector, "light_stale_sec", c.light_stale_sec);
  const auto throttle = root["throttle"];
  read_value(throttle, "speed_min", c.speed_min); read_value(throttle, "speed_max", c.speed_max);
  read_value(throttle, "launch_cap", c.launch_cap); read_value(throttle, "s_curve_cap", c.s_curve_cap);
  read_value(throttle, "fork_approach_cap", c.fork_approach_cap);
  read_value(throttle, "fork_commit_cap", c.fork_commit_cap);
  read_value(throttle, "post_fork_cap", c.post_fork_cap); read_value(throttle, "post_fork_min", c.post_fork_min);
  read_value(throttle, "ramp_up_per_cmd", c.ramp_up); read_value(throttle, "steer_slowdown", c.steer_slowdown);
  read_value(throttle, "curvature_slowdown", c.curvature_slowdown);
  const auto steering = root["steering"];
  read_value(steering, "steer_sign", c.steer_sign); read_value(steering, "lookahead_m", c.lookahead);
  read_value(steering, "wheelbase_m", c.wheelbase); read_value(steering, "lateral_scale_m", c.lateral_scale);
  read_value(steering, "max_steer_deg", c.max_steer_deg); read_value(steering, "pp_gain", c.pp_gain);
  read_value(steering, "curve_blend", c.curve_blend); read_value(steering, "rate_limit_per_cmd", c.steer_rate);
  read_value(steering, "straight_limit", c.straight_limit); read_value(steering, "s_curve_limit", c.s_curve_limit);
  read_value(steering, "fork_approach_limit", c.fork_approach_limit);
  read_value(steering, "fork_limit", c.fork_limit); read_value(steering, "post_fork_limit", c.post_fork_limit);
  read_value(steering, "lost_decay", c.lost_decay);
  const auto color = root["color_correction"];
  read_value(color, "enabled", c.color_enabled);
  read_value(color, "clahe_clip", c.color_clahe_clip);
  read_value(color, "clahe_tile", c.color_clahe_tile);
  read_value(color, "saturation_boost", c.saturation_boost);
  read_value(color, "brightness", c.brightness);
  read_value(color, "contrast", c.contrast);
  read_value(color, "saturation", c.saturation);
  read_value(color, "gamma", c.gamma);
  const auto mission = root["mission"];
  read_value(mission, "launch_min_sec", c.launch_min_sec);
  read_value(mission, "fork_commit_min_sec", c.fork_commit_min_sec);
  read_value(mission, "fork_commit_timeout_sec", c.fork_commit_timeout_sec);
  read_value(mission, "finish_min_elapsed_sec", c.finish_min_elapsed_sec);
  const auto aruco = root["aruco"];
  read_value(aruco, "target_id", c.aruco_target_id);
  return c;
}

class LaneProcessor {
public:
  explicit LaneProcessor(const Config & config) : c_(config) {
    rebuild_kernels();
  }

  void update_config(const Config & config) {
    c_ = config;
    rebuild_kernels();
  }

  LaneObs process(const cv::Mat & frame, LaneDebug * debug = nullptr) {
    LaneObs obs;
    if (frame.empty()) return obs;
    const int rw = std::clamp(c_.lane_width, 1, frame.cols);
    const int rh = std::clamp(c_.lane_height, 1, frame.rows);
    const int x0 = c_.lane_x < 0 ? (frame.cols - rw) / 2 : std::clamp(c_.lane_x, 0, frame.cols - rw);
    const int y0 = c_.lane_y < 0 ? frame.rows - rh : std::clamp(c_.lane_y, 0, frame.rows - rh);
    const cv::Mat roi = frame(cv::Rect(x0, y0, rw, rh));

    // One BGR->LAB + one CLAHE pass is shared by mask and Hough.
    cv::Mat lab;
    cv::cvtColor(roi, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> channels;
    cv::split(lab, channels);
    clahe_->apply(channels[0], channels[0]);
    cv::merge(channels, lab);
    cv::Mat road_mask;
    cv::inRange(
      lab,
      cv::Scalar(c_.lab_l_min, c_.lab_a_min, c_.lab_b_min),
      cv::Scalar(c_.lab_l_max, c_.lab_a_max, c_.lab_b_max), road_mask);
    cv::morphologyEx(road_mask, road_mask, cv::MORPH_OPEN, open_kernel_);
    cv::morphologyEx(road_mask, road_mask, cv::MORPH_CLOSE, close_kernel_);

    // Steering uses paint color rather than every brightness edge. Keeping the
    // white and yellow ranges separate is essential: one rectangular LAB band
    // cannot include neutral white and high-b yellow without also admitting the
    // dark road between them.
    cv::Mat white_mask, yellow_mask, lane_mask;
    cv::inRange(
      lab,
      cv::Scalar(c_.white_l_min, c_.white_a_min, c_.white_b_min),
      cv::Scalar(c_.white_l_max, c_.white_a_max, c_.white_b_max), white_mask);
    cv::inRange(
      lab,
      cv::Scalar(c_.yellow_l_min, c_.yellow_a_min, c_.yellow_b_min),
      cv::Scalar(c_.yellow_l_max, c_.yellow_a_max, c_.yellow_b_max), yellow_mask);
    cv::bitwise_or(white_mask, yellow_mask, lane_mask);
    cv::morphologyEx(lane_mask, lane_mask, cv::MORPH_OPEN, open_kernel_);
    cv::morphologyEx(lane_mask, lane_mask, cv::MORPH_CLOSE, close_kernel_);
    if (debug) {
      debug->roi = cv::Rect(x0, y0, rw, rh);
      debug->mask = lane_mask.clone();
    }

    auto hough_center = hough(lane_mask, debug ? &debug->hough : nullptr);
    auto near_center = hough_center;
    if (!near_center) near_center = contour_center(road_mask, c_.near_y0, 1.0);
    const auto mid_center = contour_center(road_mask, c_.mid_y0, c_.mid_y1);
    const auto far_center = contour_center(road_mask, c_.far_y0, c_.far_y1);
    if (debug) {
      debug->bands = {
        make_range(road_mask.rows, c_.near_y0, 1.0),
        make_range(road_mask.rows, c_.mid_y0, c_.mid_y1),
        make_range(road_mask.rows, c_.far_y0, c_.far_y1)};
      debug->centers = {near_center, mid_center, far_center};
    }
    if (!near_center) {
      obs.center_error = previous_error_;
      return obs;
    }

    double full_center = *near_center + x0;
    if (previous_center_ && std::abs(full_center - *previous_center_) / frame.cols > c_.max_center_jump) {
      full_center = *previous_center_;
      near_center = full_center - x0;
    }
    previous_center_ = full_center;
    obs.valid = true;
    obs.center_error = clamp((frame.cols / 2.0 - full_center) / (frame.cols / 2.0), -1.0, 1.0);
    previous_error_ = obs.center_error;
    const double far = far_center.value_or(*near_center);
    const double mid = mid_center.value_or(*near_center);
    obs.signed_curvature = clamp((*near_center - far) / frame.cols, -1.0, 1.0);
    obs.curvature = clamp(std::abs(far - *near_center) / frame.cols * 2.5, 0.0, 1.0);
    obs.curvature = std::max(obs.curvature, clamp(std::abs(mid - *near_center) / frame.cols * 2.0, 0.0, 1.0));

    const int fy0 = std::clamp(
      static_cast<int>(c_.far_y0 * road_mask.rows), 0, road_mask.rows - 1);
    const int fy1 = std::clamp(
      static_cast<int>(c_.far_y1 * road_mask.rows), fy0 + 1, road_mask.rows);
    const cv::Mat far_mask = road_mask.rowRange(fy0, fy1);
    const int half = far_mask.cols / 2;
    const double left_ratio = cv::countNonZero(far_mask.colRange(0, half)) /
      static_cast<double>(far_mask.rows * half);
    const double right_ratio = cv::countNonZero(far_mask.colRange(half, far_mask.cols)) /
      static_cast<double>(far_mask.rows * (far_mask.cols - half));
    obs.fork_seen = left_ratio >= c_.fork_area_ratio && right_ratio >= c_.fork_area_ratio;
    if (left_ratio >= c_.fork_area_ratio) obs.left_target = obs.center_error + 0.18;
    if (right_ratio >= c_.fork_area_ratio) obs.right_target = obs.center_error - 0.18;
    return obs;
  }

private:
  void rebuild_kernels() {
    clahe_ = cv::createCLAHE(c_.clahe_clip, cv::Size(c_.clahe_tile, c_.clahe_tile));
    open_kernel_ = cv::Mat::ones(std::max(1, c_.morph_open), std::max(1, c_.morph_open), CV_8U);
    close_kernel_ = cv::Mat::ones(std::max(1, c_.morph_close), std::max(1, c_.morph_close), CV_8U);
  }

  static cv::Range make_range(int height, double y0r, double y1r) {
    const int y0 = std::clamp(static_cast<int>(y0r * height), 0, height - 1);
    const int y1 = std::clamp(static_cast<int>(y1r * height), y0 + 1, height);
    return cv::Range(y0, y1);
  }

  std::optional<double> contour_center(const cv::Mat & mask, double y0r, double y1r) const {
    const auto range = make_range(mask.rows, y0r, y1r);
    cv::Mat band = mask.rowRange(range);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(band.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const double minimum = band.total() * c_.min_area_ratio;
    double sx = 0.0, area_sum = 0.0;
    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < minimum) continue;
      const auto m = cv::moments(contour);
      if (m.m00 > 0.0) { sx += (m.m10 / m.m00) * area; area_sum += area; }
    }
    if (area_sum <= 0.0) return std::nullopt;
    return sx / area_sum;
  }

  std::optional<double> hough(const cv::Mat & l_channel, HoughDebug * debug) const {
    cv::Mat blur, edges;
    cv::GaussianBlur(l_channel, blur, cv::Size(5, 5), 0.0);
    cv::Canny(blur, edges, c_.canny_low, c_.canny_high);
    const int top = std::clamp(static_cast<int>(c_.hough_top * edges.rows), 0, edges.rows - 1);
    edges.rowRange(0, top).setTo(0);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1.0, CV_PI / 180.0, c_.hough_threshold,
      c_.hough_min_length, c_.hough_max_gap);
    std::vector<std::pair<double, double>> left, right;
    for (const auto & line : lines) {
      if (line[0] == line[2]) continue;
      const double slope = static_cast<double>(line[3] - line[1]) / (line[2] - line[0]);
      if (std::abs(slope) < c_.hough_slope_min) continue;
      const double intercept = line[1] - slope * line[0];
      (slope < 0.0 ? left : right).emplace_back(slope, intercept);
    }
    if (debug) {
      debug->segments = lines;
      debug->edges = edges.clone();
      debug->top_y = top;
    }
    if (left.empty() && right.empty()) return std::nullopt;
    auto average_fit = [](const auto & fits) {
      double sm = 0.0, sb = 0.0;
      for (const auto & fit : fits) { sm += fit.first; sb += fit.second; }
      sm /= fits.size(); sb /= fits.size();
      return std::pair<double, double>{sm, sb};
    };
    auto endpoints = [top, &l_channel](const auto & fit) {
      const double bottom = static_cast<double>(l_channel.rows - 1);
      return cv::Vec4d(
        (bottom - fit.second) / fit.first, bottom,
        (top - fit.second) / fit.first, static_cast<double>(top));
    };
    std::optional<std::pair<double, double>> left_fit, right_fit;
    if (!left.empty()) left_fit = average_fit(left);
    if (!right.empty()) right_fit = average_fit(right);
    if (debug) {
      if (left_fit) debug->left_line = endpoints(*left_fit);
      if (right_fit) debug->right_line = endpoints(*right_fit);
    }
    const auto target = [top](const auto & fit) { return (top - fit.second) / fit.first; };
    if (left_fit && right_fit) return (target(*left_fit) + target(*right_fit)) / 2.0;
    if (left_fit) return target(*left_fit) + l_channel.cols * c_.assumed_lane_width / 2.0;
    return target(*right_fit) - l_channel.cols * c_.assumed_lane_width / 2.0;
  }

  Config c_;
  cv::Ptr<cv::CLAHE> clahe_;
  cv::Mat open_kernel_, close_kernel_;
  std::optional<double> previous_center_;
  double previous_error_{0.0};
};

class Controller {
public:
  explicit Controller(const Config & c) : c_(c) {}

  void update_config(const Config & config) { c_ = config; }

  Command follow(const LaneObs & lane, double cap, double steer_limit, std::optional<double> floor = std::nullopt) {
    if (!lane.valid) {
      previous_steer_ = rate(previous_steer_ * c_.lost_decay, previous_steer_, c_.steer_rate);
      previous_throttle_ = std::min(previous_throttle_, c_.speed_min);
      return {previous_throttle_, previous_steer_};
    }
    const double steer = steering(lane, steer_limit);
    return {throttle(cap, steer, lane.curvature, floor), steer};
  }

  Command fork(const LaneObs & lane, const std::string & decision) {
    LaneObs virtual_lane = lane;
    if (decision == "LEFT") virtual_lane.center_error = lane.left_target.value_or(lane.center_error + 0.18);
    if (decision == "RIGHT") virtual_lane.center_error = lane.right_target.value_or(lane.center_error - 0.18);
    const double steer = steering(virtual_lane, c_.fork_limit);
    return {throttle(c_.fork_commit_cap, steer, lane.curvature), steer};
  }

  void stop() { previous_throttle_ = 0.0; }

private:
  static double rate(double target, double previous, double delta) {
    return previous + clamp(target - previous, -delta, delta);
  }
  double steering(const LaneObs & lane, double limit) {
    const double target = clamp(lane.center_error + c_.curve_blend * lane.signed_curvature, -1.0, 1.0);
    const double alpha = std::atan2(target * c_.lateral_scale, std::max(c_.lookahead, 1e-3));
    const double delta = std::atan2(2.0 * c_.wheelbase * std::sin(alpha), std::max(c_.lookahead, 1e-3));
    double raw = c_.pp_gain * delta / (std::max(c_.max_steer_deg, 1.0) * CV_PI / 180.0);
    raw = clamp(raw, -limit, limit);
    previous_steer_ = rate(raw, previous_steer_, c_.steer_rate);
    return c_.steer_sign * previous_steer_;
  }
  double throttle(double cap, double steer, double curvature, std::optional<double> requested_floor = std::nullopt) {
    cap = clamp(cap, c_.speed_min, c_.speed_max);
    const double floor = std::min(cap, requested_floor.value_or(c_.speed_min));
    double target = clamp(cap - c_.steer_slowdown * std::abs(steer) - c_.curvature_slowdown * curvature, floor, cap);
    if (target > previous_throttle_) target = std::min(target, previous_throttle_ + c_.ramp_up);
    previous_throttle_ = target;
    return target;
  }
  Config c_;
  double previous_steer_{0.0}, previous_throttle_{0.0};
};

}  // namespace

class BisaAutonomousNode : public rclcpp::Node {
public:
  BisaAutonomousNode()
  : Node("bisa_autonomous_node"),
    config_(load_config(declare_parameter<std::string>(
      "config_file", ament_index_cpp::get_package_share_directory("bisa") + "/config/dracer_params.yaml"))),
    lane_(config_), controller_(config_) {
    route_mode_ = declare_parameter<std::string>("route_mode", "OUT");
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image/compressed");
    control_topic_ = declare_parameter<std::string>("control_topic", "/control");
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/bisa/detections");
    publish_debug_ = declare_parameter<bool>("publish_debug_image", false);
    debug_hz_ = std::max(0.1, declare_parameter<double>("debug_image_hz", 5.0));
    perception_hz_ = std::max(1.0, declare_parameter<double>("perception_hz", 20.0));
    control_hz_ = std::max(1.0, declare_parameter<double>("control_hz", 20.0));
    detection_hz_target_ = std::max(
      1.0, declare_parameter<double>("detection_hz_target", 20.0));
    config_.sign_vote_k = declare_parameter<int>("sign_vote_k", 6);
    config_.sign_vote_n = declare_parameter<int>("sign_vote_n", 10);
    config_.light_confirm_frames = declare_parameter<int>("light_confirm_frames", 8);
    declare_tunable_parameters();
    lane_.update_config(config_);
    controller_.update_config(config_);
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&BisaAutonomousNode::on_parameters, this, std::placeholders::_1));

    // Each real-time path gets an independent mutually-exclusive group.  The
    // default callback group would serialize JPEG/lane work with /control even
    // under MultiThreadedExecutor, allowing a slow frame to stall steering.
    image_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    perception_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    detection_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    debug_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    rclcpp::SubscriptionOptions image_options;
    image_options.callback_group = image_group_;
    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic_, image_qos,
      std::bind(&BisaAutonomousNode::image_callback, this, std::placeholders::_1),
      image_options);
    rclcpp::SubscriptionOptions detection_options;
    detection_options.callback_group = detection_group_;
    detection_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      detections_topic_, rclcpp::QoS(1),
      std::bind(&BisaAutonomousNode::detection_callback, this, std::placeholders::_1),
      detection_options);
    control_pub_ = create_publisher<control_msgs::msg::Control>(control_topic_, 10);
    green_pub_ = create_publisher<std_msgs::msg::Bool>("/detect_green", 10);
    red_pub_ = create_publisher<std_msgs::msg::Bool>("/detect_red", 10);
    sign_pub_ = create_publisher<std_msgs::msg::String>("/detect_sign", 10);
    aruco_pub_ = create_publisher<std_msgs::msg::String>("/detect_aruco", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "/bisa/debug/image/compressed", image_qos);
    mask_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "/bisa/debug/lane_mask/compressed", image_qos);

    perception_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / perception_hz_),
      std::bind(&BisaAutonomousNode::perception_loop, this),
      perception_group_);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_hz_),
      std::bind(&BisaAutonomousNode::control_loop, this),
      control_group_);
    if (publish_debug_) {
      debug_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / debug_hz_),
        std::bind(&BisaAutonomousNode::debug_loop, this),
        debug_group_);
    }
    state_ = route_mode_ == "LANE" ? "LANE_TEST" : "OUT_WAIT_GREEN";
    performance_started_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(),
      "C++ BISA core started: perception=%.1f Hz control=%.1f Hz debug=%.1f Hz",
      perception_hz_, control_hz_, debug_hz_);
  }

private:
  void declare_tunable_parameters() {
    config_.lane_width = declare_parameter<int>("lane_roi.width", config_.lane_width);
    config_.lane_height = declare_parameter<int>("lane_roi.height", config_.lane_height);
    config_.lane_x = declare_parameter<int>("lane_roi.x_offset", config_.lane_x);
    config_.lane_y = declare_parameter<int>("lane_roi.y_offset", config_.lane_y);
    config_.lab_l_min = declare_parameter<int>("lane.lab_l_min", config_.lab_l_min);
    config_.lab_l_max = declare_parameter<int>("lane.lab_l_max", config_.lab_l_max);
    config_.lab_a_min = declare_parameter<int>("lane.lab_a_min", config_.lab_a_min);
    config_.lab_a_max = declare_parameter<int>("lane.lab_a_max", config_.lab_a_max);
    config_.lab_b_min = declare_parameter<int>("lane.lab_b_min", config_.lab_b_min);
    config_.lab_b_max = declare_parameter<int>("lane.lab_b_max", config_.lab_b_max);
    config_.white_l_min = declare_parameter<int>("lane.white_l_min", config_.white_l_min);
    config_.white_l_max = declare_parameter<int>("lane.white_l_max", config_.white_l_max);
    config_.white_a_min = declare_parameter<int>("lane.white_a_min", config_.white_a_min);
    config_.white_a_max = declare_parameter<int>("lane.white_a_max", config_.white_a_max);
    config_.white_b_min = declare_parameter<int>("lane.white_b_min", config_.white_b_min);
    config_.white_b_max = declare_parameter<int>("lane.white_b_max", config_.white_b_max);
    config_.yellow_l_min = declare_parameter<int>("lane.yellow_l_min", config_.yellow_l_min);
    config_.yellow_l_max = declare_parameter<int>("lane.yellow_l_max", config_.yellow_l_max);
    config_.yellow_a_min = declare_parameter<int>("lane.yellow_a_min", config_.yellow_a_min);
    config_.yellow_a_max = declare_parameter<int>("lane.yellow_a_max", config_.yellow_a_max);
    config_.yellow_b_min = declare_parameter<int>("lane.yellow_b_min", config_.yellow_b_min);
    config_.yellow_b_max = declare_parameter<int>("lane.yellow_b_max", config_.yellow_b_max);
    config_.clahe_clip = declare_parameter<double>("lane.lab_clahe_clip", config_.clahe_clip);
    config_.clahe_tile = declare_parameter<int>("lane.lab_clahe_tile", config_.clahe_tile);
    config_.morph_open = declare_parameter<int>("lane.morph_open_kernel", config_.morph_open);
    config_.morph_close = declare_parameter<int>("lane.morph_close_kernel", config_.morph_close);
    config_.min_area_ratio = declare_parameter<double>(
      "lane.min_component_area_ratio", config_.min_area_ratio);
    config_.fork_area_ratio = declare_parameter<double>("lane.fork_area_ratio", config_.fork_area_ratio);
    config_.hough_top = declare_parameter<double>("lane.hough_roi_top_ratio", config_.hough_top);
    config_.canny_low = declare_parameter<int>("lane.hough_canny_low", config_.canny_low);
    config_.canny_high = declare_parameter<int>("lane.hough_canny_high", config_.canny_high);
    config_.hough_threshold = declare_parameter<int>("lane.hough_threshold", config_.hough_threshold);
    config_.hough_min_length = declare_parameter<int>(
      "lane.hough_min_line_length", config_.hough_min_length);
    config_.hough_max_gap = declare_parameter<int>("lane.hough_max_line_gap", config_.hough_max_gap);
    config_.hough_slope_min = declare_parameter<double>(
      "lane.hough_slope_min_abs", config_.hough_slope_min);

    config_.lookahead = declare_parameter<double>("steering.lookahead_m", config_.lookahead);
    config_.wheelbase = declare_parameter<double>("steering.wheelbase_m", config_.wheelbase);
    config_.lateral_scale = declare_parameter<double>("steering.lateral_scale_m", config_.lateral_scale);
    config_.max_steer_deg = declare_parameter<double>("steering.max_steer_deg", config_.max_steer_deg);
    config_.pp_gain = declare_parameter<double>("steering.pp_gain", config_.pp_gain);
    config_.curve_blend = declare_parameter<double>("steering.curve_blend", config_.curve_blend);
    config_.straight_limit = declare_parameter<double>("steering.straight_limit", config_.straight_limit);
    config_.s_curve_limit = declare_parameter<double>("steering.s_curve_limit", config_.s_curve_limit);
    config_.steer_rate = declare_parameter<double>("steering.rate_limit_per_cmd", config_.steer_rate);
    config_.steer_sign = declare_parameter<int>("steering.steer_sign", config_.steer_sign);

    config_.speed_min = declare_parameter<double>("throttle.speed_min", config_.speed_min);
    config_.speed_max = declare_parameter<double>("throttle.speed_max", config_.speed_max);
    config_.launch_cap = declare_parameter<double>("throttle.launch_cap", config_.launch_cap);
    config_.s_curve_cap = declare_parameter<double>("throttle.s_curve_cap", config_.s_curve_cap);

    config_.color_enabled = declare_parameter<bool>("color_correction.enabled", config_.color_enabled);
    config_.color_clahe_clip = declare_parameter<double>(
      "color_correction.clahe_clip", config_.color_clahe_clip);
    config_.saturation_boost = declare_parameter<double>(
      "color_correction.saturation_boost", config_.saturation_boost);
    config_.brightness = declare_parameter<int>("color_correction.brightness", config_.brightness);
    config_.contrast = declare_parameter<double>("color_correction.contrast", config_.contrast);
    config_.saturation = declare_parameter<double>("color_correction.saturation", config_.saturation);
    config_.gamma = declare_parameter<double>("color_correction.gamma", config_.gamma);
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    Config next;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      next = config_;
    }
    bool lane_changed = false;
    bool controller_changed = false;
    try {
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        lane_changed = lane_changed || name.rfind("lane.", 0) == 0 || name.rfind("lane_roi.", 0) == 0;
        controller_changed = controller_changed ||
          name.rfind("steering.", 0) == 0 || name.rfind("throttle.", 0) == 0;
        if (name == "lane_roi.width") next.lane_width = parameter.as_int();
        else if (name == "lane_roi.height") next.lane_height = parameter.as_int();
        else if (name == "lane_roi.x_offset") next.lane_x = parameter.as_int();
        else if (name == "lane_roi.y_offset") next.lane_y = parameter.as_int();
        else if (name == "lane.lab_l_min") next.lab_l_min = parameter.as_int();
        else if (name == "lane.lab_l_max") next.lab_l_max = parameter.as_int();
        else if (name == "lane.lab_a_min") next.lab_a_min = parameter.as_int();
        else if (name == "lane.lab_a_max") next.lab_a_max = parameter.as_int();
        else if (name == "lane.lab_b_min") next.lab_b_min = parameter.as_int();
        else if (name == "lane.lab_b_max") next.lab_b_max = parameter.as_int();
        else if (name == "lane.white_l_min") next.white_l_min = parameter.as_int();
        else if (name == "lane.white_l_max") next.white_l_max = parameter.as_int();
        else if (name == "lane.white_a_min") next.white_a_min = parameter.as_int();
        else if (name == "lane.white_a_max") next.white_a_max = parameter.as_int();
        else if (name == "lane.white_b_min") next.white_b_min = parameter.as_int();
        else if (name == "lane.white_b_max") next.white_b_max = parameter.as_int();
        else if (name == "lane.yellow_l_min") next.yellow_l_min = parameter.as_int();
        else if (name == "lane.yellow_l_max") next.yellow_l_max = parameter.as_int();
        else if (name == "lane.yellow_a_min") next.yellow_a_min = parameter.as_int();
        else if (name == "lane.yellow_a_max") next.yellow_a_max = parameter.as_int();
        else if (name == "lane.yellow_b_min") next.yellow_b_min = parameter.as_int();
        else if (name == "lane.yellow_b_max") next.yellow_b_max = parameter.as_int();
        else if (name == "lane.lab_clahe_clip") next.clahe_clip = parameter.as_double();
        else if (name == "lane.lab_clahe_tile") next.clahe_tile = parameter.as_int();
        else if (name == "lane.morph_open_kernel") next.morph_open = parameter.as_int();
        else if (name == "lane.morph_close_kernel") next.morph_close = parameter.as_int();
        else if (name == "lane.min_component_area_ratio") next.min_area_ratio = parameter.as_double();
        else if (name == "lane.fork_area_ratio") next.fork_area_ratio = parameter.as_double();
        else if (name == "lane.hough_roi_top_ratio") next.hough_top = parameter.as_double();
        else if (name == "lane.hough_canny_low") next.canny_low = parameter.as_int();
        else if (name == "lane.hough_canny_high") next.canny_high = parameter.as_int();
        else if (name == "lane.hough_threshold") next.hough_threshold = parameter.as_int();
        else if (name == "lane.hough_min_line_length") next.hough_min_length = parameter.as_int();
        else if (name == "lane.hough_max_line_gap") next.hough_max_gap = parameter.as_int();
        else if (name == "lane.hough_slope_min_abs") next.hough_slope_min = parameter.as_double();
        else if (name == "steering.lookahead_m") next.lookahead = parameter.as_double();
        else if (name == "steering.wheelbase_m") next.wheelbase = parameter.as_double();
        else if (name == "steering.lateral_scale_m") next.lateral_scale = parameter.as_double();
        else if (name == "steering.max_steer_deg") next.max_steer_deg = parameter.as_double();
        else if (name == "steering.pp_gain") next.pp_gain = parameter.as_double();
        else if (name == "steering.curve_blend") next.curve_blend = parameter.as_double();
        else if (name == "steering.straight_limit") next.straight_limit = parameter.as_double();
        else if (name == "steering.s_curve_limit") next.s_curve_limit = parameter.as_double();
        else if (name == "steering.rate_limit_per_cmd") next.steer_rate = parameter.as_double();
        else if (name == "steering.steer_sign") next.steer_sign = parameter.as_int();
        else if (name == "throttle.speed_min") next.speed_min = parameter.as_double();
        else if (name == "throttle.speed_max") next.speed_max = parameter.as_double();
        else if (name == "throttle.launch_cap") next.launch_cap = parameter.as_double();
        else if (name == "throttle.s_curve_cap") next.s_curve_cap = parameter.as_double();
        else if (name == "color_correction.enabled") next.color_enabled = parameter.as_bool();
        else if (name == "color_correction.clahe_clip") next.color_clahe_clip = parameter.as_double();
        else if (name == "color_correction.saturation_boost") next.saturation_boost = parameter.as_double();
        else if (name == "color_correction.brightness") next.brightness = parameter.as_int();
        else if (name == "color_correction.contrast") next.contrast = parameter.as_double();
        else if (name == "color_correction.saturation") next.saturation = parameter.as_double();
        else if (name == "color_correction.gamma") next.gamma = parameter.as_double();
      }
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & error) {
      return rcl_interfaces::msg::SetParametersResult().set__successful(false).set__reason(error.what());
    }

    const bool lab_valid =
      next.lab_l_min <= next.lab_l_max && next.lab_a_min <= next.lab_a_max &&
      next.lab_b_min <= next.lab_b_max &&
      next.white_l_min <= next.white_l_max && next.white_a_min <= next.white_a_max &&
      next.white_b_min <= next.white_b_max &&
      next.yellow_l_min <= next.yellow_l_max && next.yellow_a_min <= next.yellow_a_max &&
      next.yellow_b_min <= next.yellow_b_max;
    const bool geometry_valid = next.lane_width >= 1 && next.lane_height >= 1 && next.clahe_tile >= 1 &&
      next.morph_open >= 1 && next.morph_close >= 1 && next.lookahead > 0.0 && next.wheelbase > 0.0 &&
      next.max_steer_deg > 0.0 && next.gamma > 0.0;
    const bool speed_valid = next.speed_min >= 0.0 && next.speed_min <= next.speed_max && next.speed_max <= 1.0;
    if (!lab_valid || !geometry_valid || !speed_valid || std::abs(next.steer_sign) != 1) {
      return rcl_interfaces::msg::SetParametersResult().set__successful(false).set__reason(
        "invalid LAB bounds, ROI/kernel size, steering geometry/sign, gamma, or speed band");
    }

    if (lane_changed) {
      std::lock_guard<std::mutex> lane_lock(lane_mutex_);
      lane_.update_config(next);
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      config_ = next;
      if (controller_changed) controller_.update_config(config_);
    }
    RCLCPP_INFO(get_logger(), "applied %zu live parameter(s)", parameters.size());
    return rcl_interfaces::msg::SetParametersResult().set__successful(true);
  }

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    // Depth-1/latest-only transport: never decode or run OpenCV in the DDS
    // callback.  The dedicated 20 Hz perception timer consumes the newest
    // complete MJPEG frame and naturally drops superseded frames.
    std::lock_guard<std::mutex> lock(image_mutex_);
    pending_image_ = msg;
  }

  void perception_loop() {
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      msg = pending_image_;
    }
    if (!msg ||
      (msg->header.stamp.sec == processed_image_sec_ &&
      msg->header.stamp.nanosec == processed_image_nanosec_))
    {
      return;
    }

    const auto processing_started = std::chrono::steady_clock::now();
    cv::Mat encoded(1, static_cast<int>(msg->data.size()), CV_8U, msg->data.data());
    cv::Mat frame = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (frame.empty()) return;
    LaneDebug lane_debug;
    LaneObs observation;
    {
      std::lock_guard<std::mutex> lane_lock(lane_mutex_);
      observation = lane_.process(frame, publish_debug_ ? &lane_debug : nullptr);
    }
    bool marker = false;
    int aruco_target_id = 3;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      aruco_target_id = config_.aruco_target_id;
    }
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50), corners, ids);
    marker = std::find(ids.begin(), ids.end(), aruco_target_id) != ids.end();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_lane_ = observation;
      target_marker_ = marker;
      marker_ids_ = std::move(ids);
      marker_corners_ = std::move(corners);
      if (publish_debug_) {
        latest_frame_ = frame;
        latest_lane_debug_ = std::move(lane_debug);
        frame_history_.push_back(
          StampedFrame{msg->header.stamp.sec, msg->header.stamp.nanosec, frame});
        while (frame_history_.size() > 40U) frame_history_.pop_front();
        if (msg->header.stamp.sec == detection_source_sec_ &&
          msg->header.stamp.nanosec == detection_source_nanosec_)
        {
          detection_frame_ = frame;
          debug_detections_ = detections_;
          detection_stamp_ = rclcpp::Time(
            msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
        }
      }
    }
    processed_image_sec_ = msg->header.stamp.sec;
    processed_image_nanosec_ = msg->header.stamp.nanosec;

    const auto processing_finished = std::chrono::steady_clock::now();
    const double processing_ms = std::chrono::duration<double, std::milli>(
      processing_finished - processing_started).count();
    if (perception_count_ == 0U) performance_started_ = processing_started;
    ++perception_count_;
    perception_ms_sum_ += processing_ms;
    perception_ms_max_ = std::max(perception_ms_max_, processing_ms);
    const double report_sec = std::chrono::duration<double>(
      processing_finished - performance_started_).count();
    if (report_sec >= 3.0) {
      RCLCPP_INFO(
        get_logger(), "C++ perception: %.1f Hz, %.1f ms avg, %.1f ms max (target %.1f Hz)",
        perception_count_ / report_sec,
        perception_ms_sum_ / std::max<uint64_t>(perception_count_, 1U),
        perception_ms_max_, perception_hz_);
      performance_started_ = processing_finished;
      perception_count_ = 0;
      perception_ms_sum_ = 0.0;
      perception_ms_max_ = 0.0;
    }
  }

  void detection_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 5) return;
    const auto detection_now = std::chrono::steady_clock::now();
    if (detection_count_ == 0U) detection_report_started_ = detection_now;
    ++detection_count_;
    const uint64_t sequence = static_cast<uint64_t>(msg->data[0]);
    const int32_t source_sec = static_cast<int32_t>(msg->data[1]);
    const uint32_t source_nanosec = static_cast<uint32_t>(msg->data[2]);
    const int light = static_cast<int>(msg->data[3]);
    const int count = std::max(0, static_cast<int>(msg->data[4]));
    std::vector<Detection> detections;
    bool left = false, right = false;
    for (int i = 0; i < count; ++i) {
      const std::size_t base = 5 + static_cast<std::size_t>(i) * 6;
      if (base + 5 >= msg->data.size()) break;
      Detection d;
      d.class_id = static_cast<int>(msg->data[base]);
      d.confidence = msg->data[base + 1];
      const float x1 = msg->data[base + 2], y1 = msg->data[base + 3];
      d.box = cv::Rect2f(x1, y1, msg->data[base + 4] - x1, msg->data[base + 5] - y1);
      left = left || d.class_id == 2; right = right || d.class_id == 3;
      detections.push_back(d);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    detection_sequence_ = sequence;
    light_state_ = light;
    light_received_ = now();
    detections_ = detections;
    detection_source_sec_ = source_sec;
    detection_source_nanosec_ = source_nanosec;
    for (auto it = frame_history_.rbegin(); it != frame_history_.rend(); ++it) {
      if (it->sec == source_sec && it->nanosec == source_nanosec) {
        detection_frame_ = it->image;
        debug_detections_ = detections;
        detection_stamp_ = rclcpp::Time(source_sec, source_nanosec, RCL_ROS_TIME);
        break;
      }
    }
    sign_history_.push_back((left ? 1 : 0) | (right ? 2 : 0));
    while (static_cast<int>(sign_history_.size()) > config_.sign_vote_n) sign_history_.pop_front();
    const double report_sec = std::chrono::duration<double>(
      detection_now - detection_report_started_).count();
    if (report_sec >= 3.0) {
      const double actual_hz = detection_count_ / report_sec;
      if (actual_hz + 0.5 < detection_hz_target_) {
        RCLCPP_WARN(
          get_logger(), "detector input: %.1f Hz below %.1f Hz target",
          actual_hz, detection_hz_target_);
      } else {
        RCLCPP_INFO(
          get_logger(), "detector input: %.1f Hz (target %.1f Hz)",
          actual_hz, detection_hz_target_);
      }
      detection_count_ = 0;
    }
  }

  std::optional<std::string> sign_decision() const {
    int left = 0, right = 0;
    for (int bits : sign_history_) { left += bits & 1 ? 1 : 0; right += bits & 2 ? 1 : 0; }
    if (left >= config_.sign_vote_k && left > right) return "LEFT";
    if (right >= config_.sign_vote_k && right > left) return "RIGHT";
    return std::nullopt;
  }

  void update_light(double now_sec) {
    const double age = (now() - light_received_).seconds();
    const int fresh = age >= 0.0 && age <= config_.light_stale_sec ? light_state_ : 0;
    if (fresh == 0) { green_streak_ = red_streak_ = 0; return; }
    if (detection_sequence_ == last_light_sequence_) return;
    last_light_sequence_ = detection_sequence_;
    green_streak_ = fresh == 1 ? green_streak_ + 1 : 0;
    red_streak_ = fresh == 2 ? red_streak_ + 1 : 0;
    (void)now_sec;
  }

  void transition(const std::string & next, double now_sec) { state_ = next; entered_ = now_sec; }

  Command step(const LaneObs & lane, double now_sec) {
    if (route_mode_ == "LANE") return controller_.follow(lane, config_.speed_max, config_.s_curve_limit);
    if (started_ <= 0.0) { started_ = entered_ = now_sec; }
    if (state_ != "OUT_WAIT_GREEN" && state_ != "OUT_FINISH_STOP" &&
      now_sec - started_ >= config_.finish_min_elapsed_sec && red_streak_ >= config_.light_confirm_frames) {
      transition("OUT_FINISH_STOP", now_sec);
    }
    if (state_ == "OUT_WAIT_GREEN") {
      if (green_streak_ >= config_.light_confirm_frames) { started_ = now_sec; transition("OUT_LAUNCH", now_sec); }
      return {};
    }
    if (state_ == "OUT_LAUNCH") {
      auto cmd = controller_.follow(lane, config_.launch_cap, config_.straight_limit);
      if (now_sec - entered_ > config_.launch_min_sec) transition("OUT_S_CURVE", now_sec);
      return cmd;
    }
    if (state_ == "OUT_S_CURVE") {
      auto cmd = controller_.follow(lane, config_.s_curve_cap, config_.s_curve_limit);
      if (sign_decision() || lane.fork_seen) transition("OUT_FORK_APPROACH", now_sec);
      return cmd;
    }
    if (state_ == "OUT_FORK_APPROACH") {
      auto cmd = controller_.follow(lane, config_.fork_approach_cap, config_.fork_approach_limit);
      if (auto decision = sign_decision()) { fork_decision_ = *decision; transition("OUT_FORK_COMMIT", now_sec); }
      return cmd;
    }
    if (state_ == "OUT_FORK_COMMIT") {
      auto cmd = controller_.fork(lane, fork_decision_);
      const bool reacquired = lane.valid && !lane.fork_seen && std::abs(lane.center_error) < 0.45;
      if ((reacquired && now_sec - entered_ > config_.fork_commit_min_sec) ||
        now_sec - entered_ > config_.fork_commit_timeout_sec) transition("OUT_POST_FORK", now_sec);
      return cmd;
    }
    if (state_ == "OUT_POST_FORK") {
      return controller_.follow(lane, config_.post_fork_cap, config_.post_fork_limit, config_.post_fork_min);
    }
    controller_.stop();
    return {};
  }

  void control_loop() {
    LaneObs lane;
    bool marker;
    int light;
    std::vector<int> markers;
    Command cmd;
    double speed_max;
    std::optional<std::string> decision;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double now_sec = now().seconds();
      update_light(now_sec);
      lane = latest_lane_; marker = target_marker_; markers = marker_ids_;
      const double light_age = (now() - light_received_).seconds();
      light = light_age >= 0.0 && light_age <= config_.light_stale_sec ? light_state_ : 0;
      cmd = marker ? Command{} : step(lane, now_sec);
      if (marker) controller_.stop();
      last_command_ = cmd;
      decision = sign_decision();
      speed_max = config_.speed_max;
    }
    control_msgs::msg::Control output;
    output.header.stamp = now();
    output.steering = static_cast<float>(clamp(cmd.steering, -1.0, 1.0));
    output.throttle = static_cast<float>(clamp(cmd.throttle, 0.0, speed_max));
    control_pub_->publish(output);
    green_pub_->publish(std_msgs::msg::Bool().set__data(light == 1));
    red_pub_->publish(std_msgs::msg::Bool().set__data(light == 2));
    sign_pub_->publish(std_msgs::msg::String().set__data(decision.value_or("none")));
    std::string marker_text = markers.empty() ? "none" : "ids=";
    for (int id : markers) marker_text += std::to_string(id) + ",";
    aruco_pub_->publish(std_msgs::msg::String().set__data(marker_text));
  }

  static void publish_jpeg(
    const cv::Mat & image,
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & publisher,
    const rclcpp::Time & stamp) {
    if (image.empty()) return;
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = stamp;
    msg.format = "jpeg";
    cv::imencode(".jpg", image, msg.data, {cv::IMWRITE_JPEG_QUALITY, 80});
    publisher->publish(msg);
  }

  static void put_outlined_text(
    cv::Mat & image, const std::string & text, const cv::Point & origin,
    double scale = 0.55, const cv::Scalar & color = cv::Scalar(255, 255, 255))
  {
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
      cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
      color, 1, cv::LINE_AA);
  }

  static cv::Point steering_tip(const cv::Point & origin, double steering, int length) {
    constexpr double max_steer_degrees = 50.0;
    const double theta = clamp(steering, -1.0, 1.0) * max_steer_degrees * CV_PI / 180.0;
    return cv::Point(
      cvRound(origin.x - length * std::sin(theta)),
      cvRound(origin.y - length * std::cos(theta)));
  }

  static std::string detection_name(int class_id) {
    static const std::array<std::string, 4> names = {
      "traffic_red", "traffic_green", "sign_left", "sign_right"};
    return class_id >= 0 && class_id < static_cast<int>(names.size()) ?
      names[class_id] : "unknown";
  }

  cv::Mat corrected_debug_frame(const cv::Mat & input, const Config & config) {
    cv::Mat lab;
    cv::cvtColor(input, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> channels;
    cv::split(lab, channels);
    const double clip = std::max(config.color_clahe_clip, 0.01);
    const int tile = std::max(config.color_clahe_tile, 1);
    if (!debug_clahe_ || std::abs(clip - debug_clahe_clip_) >= 1e-6 || tile != debug_clahe_tile_) {
      debug_clahe_ = cv::createCLAHE(clip, cv::Size(tile, tile));
      debug_clahe_clip_ = clip;
      debug_clahe_tile_ = tile;
    }
    debug_clahe_->apply(channels[0], channels[0]);
    cv::merge(channels, lab);
    cv::Mat output;
    cv::cvtColor(lab, output, cv::COLOR_Lab2BGR);

    auto scale_saturation = [](cv::Mat & bgr, double factor) {
      if (std::abs(factor - 1.0) < 1e-3) return;
      cv::Mat hsv;
      cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
      std::vector<cv::Mat> hsv_channels;
      cv::split(hsv, hsv_channels);
      hsv_channels[1].convertTo(hsv_channels[1], CV_8U, std::max(factor, 0.0));
      cv::merge(hsv_channels, hsv);
      cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    };
    // Both saturation controls are multiplicative, so one HSV round trip is
    // equivalent and avoids two full-frame color conversions at debug rate.
    scale_saturation(output, config.saturation_boost * config.saturation);
    if (std::abs(config.contrast - 1.0) >= 1e-3 || config.brightness != 0) {
      cv::convertScaleAbs(
        output, output, std::max(config.contrast, 0.0), config.brightness);
    }
    if (std::abs(config.gamma - 1.0) >= 1e-3) {
      cv::Mat lut(1, 256, CV_8U);
      const double inverse_gamma = 1.0 / std::max(config.gamma, 0.1);
      for (int i = 0; i < 256; ++i) {
        lut.at<uint8_t>(i) = cv::saturate_cast<uint8_t>(
          std::pow(i / 255.0, inverse_gamma) * 255.0);
      }
      cv::LUT(output, lut, output);
    }
    return output;
  }

  static void draw_lane_debug(
    cv::Mat & frame, const LaneDebug & debug, const LaneObs & lane, const Command & cmd)
  {
    if (debug.roi.width <= 0 || debug.roi.height <= 0) return;
    const cv::Scalar roi_color(255, 128, 0);
    const cv::Scalar lane_color(0, 255, 0);
    cv::rectangle(frame, debug.roi, roi_color, 2);
    put_outlined_text(
      frame,
      "lane ROI " + std::to_string(debug.roi.width) + "x" + std::to_string(debug.roi.height),
      cv::Point(debug.roi.x + 4, std::max(16, debug.roi.y + 18)), 0.5, roi_color);

    auto draw_average = [&](const std::optional<cv::Vec4d> & line) {
      if (!line) return;
      cv::line(frame,
        cv::Point(cvRound((*line)[0]) + debug.roi.x, cvRound((*line)[1]) + debug.roi.y),
        cv::Point(cvRound((*line)[2]) + debug.roi.x, cvRound((*line)[3]) + debug.roi.y),
        lane_color, 5, cv::LINE_AA);
    };
    draw_average(debug.hough.left_line);
    draw_average(debug.hough.right_line);

    const int center_x = frame.cols / 2;
    cv::line(frame, cv::Point(center_x, 0), cv::Point(center_x, frame.rows - 1),
      cv::Scalar(255, 255, 0), 1);
    const auto & near_band = debug.bands[0];
    const int marker_y = debug.roi.y + (near_band.start + near_band.end) / 2;
    cv::circle(frame, cv::Point(center_x, marker_y), 7, cv::Scalar(0, 0, 255), -1);
    if (debug.centers[0]) {
      cv::circle(frame,
        cv::Point(debug.roi.x + cvRound(*debug.centers[0]), marker_y),
        7, lane_color, -1);
    }
    const cv::Point steer_origin(center_x, frame.rows - 1);
    cv::arrowedLine(frame, steer_origin,
      steering_tip(steer_origin, cmd.steering, frame.rows / 2),
      cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0, 0.15);
    (void)lane;
  }

  static cv::Mat make_lane_mask_view(
    const LaneDebug & debug, const LaneObs & lane, const Command & cmd, int frame_width)
  {
    if (debug.mask.empty()) return {};
    cv::Mat view;
    cv::cvtColor(debug.mask, view, cv::COLOR_GRAY2BGR);
    if (!debug.hough.edges.empty() && debug.hough.edges.size() == debug.mask.size()) {
      view.setTo(cv::Scalar(255, 120, 0), debug.hough.edges);
    }
    const std::array<cv::Scalar, 3> colors = {
      cv::Scalar(0, 255, 255), cv::Scalar(0, 200, 255), cv::Scalar(255, 200, 0)};
    for (std::size_t i = 0; i < debug.bands.size(); ++i) {
      const auto & band = debug.bands[i];
      cv::line(view, cv::Point(0, band.start), cv::Point(view.cols - 1, band.start), colors[i], 1);
      cv::line(view, cv::Point(0, band.end - 1), cv::Point(view.cols - 1, band.end - 1), colors[i], 1);
      if (debug.centers[i]) {
        cv::circle(view,
          cv::Point(cvRound(*debug.centers[i]), (band.start + band.end) / 2), 6, colors[i], -1);
      }
    }
    for (const auto & segment : debug.hough.segments) {
      cv::line(view, cv::Point(segment[0], segment[1]), cv::Point(segment[2], segment[3]),
        cv::Scalar(0, 160, 0), 1);
    }
    auto draw_average = [&](const std::optional<cv::Vec4d> & line) {
      if (!line) return;
      cv::line(view,
        cv::Point(cvRound((*line)[0]), cvRound((*line)[1])),
        cv::Point(cvRound((*line)[2]), cvRound((*line)[3])),
        cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    };
    draw_average(debug.hough.left_line);
    draw_average(debug.hough.right_line);
    cv::line(view, cv::Point(0, debug.hough.top_y),
      cv::Point(view.cols - 1, debug.hough.top_y), cv::Scalar(0, 160, 0), 1);

    const int vehicle_x = std::clamp(frame_width / 2 - debug.roi.x, 0, view.cols - 1);
    cv::line(view, cv::Point(vehicle_x, 0), cv::Point(vehicle_x, view.rows - 1),
      cv::Scalar(255, 255, 0), 1);
    const cv::Point steer_origin(vehicle_x, view.rows - 1);
    cv::arrowedLine(view, steer_origin,
      steering_tip(steer_origin, cmd.steering, static_cast<int>(view.rows * 0.45)),
      cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.2);
    put_outlined_text(view, "mask=WHITE|YELLOW " + std::to_string(view.cols) + "x" +
      std::to_string(view.rows),
      cv::Point(8, 20));
    std::ostringstream status;
    status.setf(std::ios::fixed); status.precision(2);
    status << "err=" << std::showpos << lane.center_error << " steer=" << cmd.steering;
    put_outlined_text(view, status.str(), cv::Point(8, 42));
    return view;
  }

  void debug_loop() {
    cv::Mat frame;
    LaneDebug lane_debug;
    LaneObs lane;
    Command cmd;
    std::vector<Detection> detections;
    std::vector<int> markers;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::string state;
    int light{0};
    Config config;
    rclcpp::Time frame_stamp{0, 0, RCL_ROS_TIME};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (latest_frame_.empty()) return;
      lane_debug = latest_lane_debug_; lane = latest_lane_;
      cmd = last_command_; state = state_;
      markers = marker_ids_; marker_corners = marker_corners_;
      light = light_state_; config = config_;
      if (!detection_frame_.empty()) {
        frame = detection_frame_.clone();
        detections = debug_detections_;
        frame_stamp = detection_stamp_;
      } else {
        // Never put stale boxes on a newer frame when an exact match is not
        // available.  The live image remains useful, simply without boxes.
        frame = latest_frame_.clone();
        detections.clear();
        frame_stamp = now();
      }
    }
    frame = corrected_debug_frame(frame, config);
    draw_lane_debug(frame, lane_debug, lane, cmd);
    for (const auto & d : detections) {
      const cv::Scalar color = d.class_id == 0 ? cv::Scalar(0, 0, 255) :
        d.class_id == 1 ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 180, 0);
      cv::rectangle(frame, d.box, color, 2);
      std::ostringstream label;
      label.setf(std::ios::fixed); label.precision(2);
      label << detection_name(d.class_id) << " " << d.confidence;
      put_outlined_text(frame, label.str(),
        cv::Point(cvRound(d.box.x), std::max(14, cvRound(d.box.y) - 5)), 0.5, color);
    }
    for (std::size_t i = 0; i < marker_corners.size(); ++i) {
      const bool target = i < markers.size() && markers[i] == config.aruco_target_id;
      const cv::Scalar color = target ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 0, 255);
      cv::polylines(frame, marker_corners[i], true, color, target ? 3 : 2, cv::LINE_AA);
      if (!marker_corners[i].empty() && i < markers.size()) {
        put_outlined_text(frame, "ID " + std::to_string(markers[i]), marker_corners[i][0], 0.55, color);
      }
    }
    put_outlined_text(frame, "state=" + state, cv::Point(8, 22));
    std::ostringstream command_text;
    command_text.setf(std::ios::fixed); command_text.precision(2);
    command_text << "thr=" << cmd.throttle << " steer=" << std::showpos << cmd.steering;
    put_outlined_text(frame, command_text.str(), cv::Point(8, 44));
    put_outlined_text(frame, "light=" + std::string(light == 1 ? "GREEN" : light == 2 ? "RED" : "none"),
      cv::Point(8, frame.rows - 12), 0.65,
      light == 1 ? cv::Scalar(0, 255, 0) : light == 2 ? cv::Scalar(0, 0, 255) : cv::Scalar(170, 170, 170));
    publish_jpeg(frame, debug_pub_, frame_stamp);
    const cv::Mat mask_view = make_lane_mask_view(lane_debug, lane, cmd, frame.cols);
    publish_jpeg(mask_view, mask_pub_, now());
  }

  Config config_;
  LaneProcessor lane_;
  Controller controller_;
  std::mutex mutex_, image_mutex_, lane_mutex_;
  sensor_msgs::msg::CompressedImage::SharedPtr pending_image_;
  LaneObs latest_lane_;
  LaneDebug latest_lane_debug_;
  cv::Mat latest_frame_, detection_frame_;
  std::vector<Detection> detections_, debug_detections_;
  std::deque<StampedFrame> frame_history_;
  std::vector<int> marker_ids_;
  std::vector<std::vector<cv::Point2f>> marker_corners_;
  std::deque<int> sign_history_;
  Command last_command_;
  bool target_marker_{false}, publish_debug_{false};
  int light_state_{0}, green_streak_{0}, red_streak_{0};
  uint64_t detection_sequence_{0}, last_light_sequence_{0};
  int32_t detection_source_sec_{0};
  uint32_t detection_source_nanosec_{0};
  int32_t processed_image_sec_{0};
  uint32_t processed_image_nanosec_{0};
  rclcpp::Time light_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time detection_stamp_{0, 0, RCL_ROS_TIME};
  std::string state_, fork_decision_, route_mode_, image_topic_, control_topic_, detections_topic_;
  double started_{0.0}, entered_{0.0}, debug_hz_{5.0}, perception_hz_{20.0}, control_hz_{20.0};
  double detection_hz_target_{20.0};
  uint64_t perception_count_{0}, detection_count_{0};
  double perception_ms_sum_{0.0}, perception_ms_max_{0.0};
  std::chrono::steady_clock::time_point performance_started_;
  std::chrono::steady_clock::time_point detection_report_started_;

  rclcpp::CallbackGroup::SharedPtr image_group_, perception_group_, detection_group_;
  rclcpp::CallbackGroup::SharedPtr control_group_, debug_group_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<control_msgs::msg::Control>::SharedPtr control_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr green_pub_, red_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sign_pub_, aruco_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_, mask_pub_;
  rclcpp::TimerBase::SharedPtr perception_timer_, control_timer_, debug_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  cv::Ptr<cv::CLAHE> debug_clahe_;
  double debug_clahe_clip_{-1.0};
  int debug_clahe_tile_{-1};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  auto node = std::make_shared<BisaAutonomousNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
