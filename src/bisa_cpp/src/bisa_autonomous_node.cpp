#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
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

double evaluate_curve(const cv::Vec3d & curve, double y, int top, int bottom) {
  const double span = std::max(1, bottom - top);
  const double t = (y - top) / span;
  return curve[0] * t * t + curve[1] * t + curve[2];
}

enum class RouteMode {OUT, IN, LANE};
enum class ForkDirection {NONE, LEFT, RIGHT};

RouteMode parse_route_mode(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char ch) {return static_cast<char>(std::toupper(ch));});
  if (value == "IN") return RouteMode::IN;
  if (value == "LANE" || value == "LANE_TEST" || value == "TEST") return RouteMode::LANE;
  return RouteMode::OUT;
}

constexpr bool should_latch_red(
  bool has_started, bool red_stop_armed, int light, int streak, int required)
{
  return has_started && red_stop_armed && light == 2 && streak >= required;
}

constexpr int kRedConfirmFrames = 3;
static_assert(
  should_latch_red(true, true, 2, kRedConfirmFrames, kRedConfirmFrames) &&
  !should_latch_red(true, false, 2, kRedConfirmFrames, kRedConfirmFrames) &&
  !should_latch_red(true, true, 2, kRedConfirmFrames - 1, kRedConfirmFrames) &&
  !should_latch_red(false, true, 2, kRedConfirmFrames, kRedConfirmFrames));

enum class MissionState {
  LANE_TEST,
  OUT_WAIT_GREEN,
  OUT_TO_FORK,
  OUT_SIGN_APPROACH,
  OUT_SIGN_VOTE_STOP,
  OUT_FORK_SIGN_ADVANCE,
  OUT_FORK_COMMIT,
  OUT_RESUME,
  IN_WAIT_GREEN,
  IN_ENTRY,
  IN_LAP,
  IN_EXIT,
  IN_RESUME,
};

const char * state_name(MissionState state) {
  switch (state) {
    case MissionState::LANE_TEST: return "LANE_TEST";
    case MissionState::OUT_WAIT_GREEN: return "OUT_WAIT_GREEN";
    case MissionState::OUT_TO_FORK: return "OUT_TO_FORK";
    case MissionState::OUT_SIGN_APPROACH: return "OUT_SIGN_APPROACH";
    case MissionState::OUT_SIGN_VOTE_STOP: return "OUT_SIGN_VOTE_STOP";
    case MissionState::OUT_FORK_SIGN_ADVANCE: return "OUT_FORK_SIGN_ADVANCE";
    case MissionState::OUT_FORK_COMMIT: return "OUT_FORK_COMMIT";
    case MissionState::OUT_RESUME: return "OUT_RESUME";
    case MissionState::IN_WAIT_GREEN: return "IN_WAIT_GREEN";
    case MissionState::IN_ENTRY: return "IN_ENTRY";
    case MissionState::IN_LAP: return "IN_LAP";
    case MissionState::IN_EXIT: return "IN_EXIT";
    case MissionState::IN_RESUME: return "IN_RESUME";
  }
  return "UNKNOWN";
}

struct LaneObs {
  bool valid{false};
  bool both_lanes{false};
  bool branch_pair_selected{false};
  bool branch_scene_ambiguous{false};
  double center_error{0.0};
  double curvature{0.0};
  double signed_curvature{0.0};
  bool fork_seen{false};
  std::optional<double> left_target;
  std::optional<double> right_target;
};

struct HoughDebug {
  std::vector<cv::Vec4i> segments;
  std::vector<cv::Vec4i> selected_segments;
  std::optional<cv::Vec3d> left_curve;
  std::optional<cv::Vec3d> right_curve;
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
  bool out_white_only{true};
  double fork_target_y0{0.05}, fork_target_y1{0.45};
  double fork_target_min_area_ratio{0.002};
  double clahe_clip{2.0};
  int clahe_tile{8};
  int morph_open{3}, morph_close{5};
  double min_area_ratio{0.006};
  double fork_area_ratio{0.035};
  double near_y0{0.70}, mid_y0{0.52}, mid_y1{0.75}, far_y0{0.32}, far_y1{0.58};
  double hough_top{0.45}, hough_curve_top{0.62}, hough_curvature_smoothing{0.25};
  int canny_low{50}, canny_high{150}, hough_threshold{38};
  int hough_min_length{30}, hough_max_gap{290};
  double hough_slope_min{0.22}, assumed_lane_width{0.62};
  double lane_width_min{0.10}, lane_width_max{0.70}, lane_width_smoothing{0.20};
  double single_lane_switch_margin{0.08}, max_center_jump{0.35};
  double speed_min{0.20}, speed_max{0.30};
  double launch_cap{0.24}, s_curve_cap{0.24}, fork_approach_cap{0.24};
  double fork_commit_cap{0.22}, post_fork_cap{0.30}, post_fork_min{0.20};
  double ramp_up{0.015}, steer_slowdown{0.22}, curvature_slowdown{0.08};
  double straight_steer_deadband{0.05}, straight_curvature_deadband{0.05};
  int steer_sign{1};
  double lookahead{0.60}, curve_lookahead_min{0.38};
  double curve_response_power{0.65}, curve_steer_boost{0.20}, fork_curve_scale{0.25};
  double fork_forced_error{0.45};
  double wheelbase{0.17}, lateral_scale{0.30};
  double max_steer_deg{30.0}, pp_gain{1.0}, curve_blend{1.0};
  double steer_rate{0.12}, straight_limit{0.60}, s_curve_limit{0.95};
  double fork_approach_limit{0.75}, fork_limit{0.95}, post_fork_limit{0.90};
  double lost_decay{0.70};
  bool color_enabled{true};
  double color_clahe_clip{2.0};
  int color_clahe_tile{8};
  double saturation_boost{1.5};
  int brightness{0};
  double contrast{1.0}, saturation{1.0}, gamma{1.0};
  int sign_vote_k{6}, sign_vote_n{10}, light_confirm_frames{8};
  double light_stale_sec{0.75};
  double sign_stop_delay_sec{1.0};
  double fork_sign_advance_sec{1.5};
  double fork_commit_min_sec{0.8}, fork_commit_timeout_sec{1.8};
  int aruco_target_id{3}, aruco_confirm_frames{2}, aruco_clear_frames{3};
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
  read_value(lane, "out_white_only", c.out_white_only);
  read_value(lane, "fork_target_y0", c.fork_target_y0);
  read_value(lane, "fork_target_y1", c.fork_target_y1);
  read_value(lane, "fork_target_min_area_ratio", c.fork_target_min_area_ratio);
  read_value(lane, "lab_clahe_clip", c.clahe_clip); read_value(lane, "lab_clahe_tile", c.clahe_tile);
  read_value(lane, "morph_open_kernel", c.morph_open); read_value(lane, "morph_close_kernel", c.morph_close);
  read_value(lane, "min_component_area_ratio", c.min_area_ratio);
  read_value(lane, "fork_area_ratio", c.fork_area_ratio);
  read_value(lane, "hough_roi_top_ratio", c.hough_top);
  read_value(lane, "hough_curve_top_ratio", c.hough_curve_top);
  read_value(lane, "hough_curvature_smoothing", c.hough_curvature_smoothing);
  read_value(lane, "hough_canny_low", c.canny_low); read_value(lane, "hough_canny_high", c.canny_high);
  read_value(lane, "hough_threshold", c.hough_threshold);
  read_value(lane, "hough_min_line_length", c.hough_min_length);
  read_value(lane, "hough_max_line_gap", c.hough_max_gap);
  read_value(lane, "hough_slope_min_abs", c.hough_slope_min);
  read_value(lane, "assumed_lane_width_ratio", c.assumed_lane_width);
  read_value(lane, "lane_width_min_ratio", c.lane_width_min);
  read_value(lane, "lane_width_max_ratio", c.lane_width_max);
  read_value(lane, "lane_width_smoothing", c.lane_width_smoothing);
  read_value(lane, "single_lane_switch_margin_ratio", c.single_lane_switch_margin);
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
  read_value(throttle, "straight_steer_deadband", c.straight_steer_deadband);
  read_value(throttle, "straight_curvature_deadband", c.straight_curvature_deadband);
  const auto steering = root["steering"];
  read_value(steering, "steer_sign", c.steer_sign); read_value(steering, "lookahead_m", c.lookahead);
  read_value(steering, "curve_lookahead_min_m", c.curve_lookahead_min);
  read_value(steering, "curve_response_power", c.curve_response_power);
  read_value(steering, "curve_steer_boost", c.curve_steer_boost);
  read_value(steering, "fork_curve_scale", c.fork_curve_scale);
  read_value(steering, "fork_forced_error", c.fork_forced_error);
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
  read_value(mission, "sign_stop_delay_sec", c.sign_stop_delay_sec);
  read_value(mission, "fork_sign_advance_sec", c.fork_sign_advance_sec);
  read_value(mission, "fork_commit_min_sec", c.fork_commit_min_sec);
  read_value(mission, "fork_commit_timeout_sec", c.fork_commit_timeout_sec);
  const auto aruco = root["aruco"];
  read_value(aruco, "target_id", c.aruco_target_id);
  read_value(aruco, "confirm_frames", c.aruco_confirm_frames);
  read_value(aruco, "clear_frames", c.aruco_clear_frames);
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

  void reset_fork_history() {
    // Discard the center, boundary identity, curvature, and width learned
    // before/inside the fork.  The next frame must classify the new lane from
    // current pixels without the previous turn angle or width pulling the fit.
    previous_center_.reset();
    previous_left_target_.reset();
    previous_right_target_.reset();
    previous_single_is_left_.reset();
    tracked_lane_width_.reset();
    filtered_curvature_ = 0.0;
    previous_error_ = 0.0;
  }

  LaneObs process(
    const cv::Mat & frame, bool include_yellow,
    ForkDirection fork_direction = ForkDirection::NONE, LaneDebug * debug = nullptr)
  {
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
    cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, open_kernel_);
    cv::morphologyEx(white_mask, white_mask, cv::MORPH_CLOSE, close_kernel_);
    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_OPEN, open_kernel_);
    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_CLOSE, close_kernel_);
    if (include_yellow) cv::bitwise_or(white_mask, yellow_mask, lane_mask);
    else lane_mask = white_mask;
    if (debug) {
      debug->roi = cv::Rect(x0, y0, rw, rh);
      debug->mask = lane_mask.clone();
    }

    // Center, curvature, steering feed-forward, and speed scheduling must all
    // use the same color-selected mask.  In OUT this is white-only, so the
    // yellow IN split cannot leak back into steering through curvature even
    // when the Hough center itself is correctly fitted to white paint.
    const auto steering_near_center = contour_center(lane_mask, c_.near_y0, 1.0);
    const auto mid_center = contour_center(lane_mask, c_.mid_y0, c_.mid_y1);
    const auto far_center = contour_center(lane_mask, c_.far_y0, c_.far_y1);
    double rough_curvature = filtered_curvature_;
    if (steering_near_center) {
      rough_curvature = 0.0;
      if (far_center) {
        rough_curvature = std::max(rough_curvature,
          clamp(std::abs(*far_center - *steering_near_center) / frame.cols * 2.5, 0.0, 1.0));
      }
      if (mid_center) {
        rough_curvature = std::max(rough_curvature,
          clamp(std::abs(*mid_center - *steering_near_center) / frame.cols * 2.0, 0.0, 1.0));
      }
    }
    const double smoothing = clamp(c_.hough_curvature_smoothing, 0.0, 1.0);
    filtered_curvature_ += smoothing * (rough_curvature - filtered_curvature_);

    const double vehicle_x = frame.cols / 2.0 - x0;
    const std::optional<double> previous_center_local = previous_center_ ?
      std::optional<double>(*previous_center_ - x0) : std::nullopt;
    bool both_lanes = false;
    bool branch_pair_selected = false;
    bool branch_scene_ambiguous = false;
    auto hough_center = hough(
      lane_mask, vehicle_x, filtered_curvature_, previous_center_local,
      fork_direction, &both_lanes, &branch_pair_selected, &branch_scene_ambiguous,
      debug ? &debug->hough : nullptr);
    auto near_center = hough_center;
    // OUT deliberately has no road-mask fallback: when white paint is absent,
    // hold the previous command instead of snapping toward the yellow IN split.
    if (!near_center && include_yellow) near_center = steering_near_center;
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
    obs.both_lanes = both_lanes;
    obs.branch_pair_selected = branch_pair_selected;
    obs.branch_scene_ambiguous = branch_scene_ambiguous;
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
    if (obs.fork_seen) {
      obs.left_target = branch_target_error(white_mask, true, x0, frame.cols);
      obs.right_target = branch_target_error(white_mask, false, x0, frame.cols);
    }
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

  std::optional<double> branch_target_error(
    const cv::Mat & white_mask, bool left, int roi_x, int frame_width) const
  {
    const int y0 = std::clamp(
      static_cast<int>(c_.fork_target_y0 * white_mask.rows), 0, white_mask.rows - 1);
    const int y1 = std::clamp(
      static_cast<int>(c_.fork_target_y1 * white_mask.rows), y0 + 1, white_mask.rows);
    const int split = white_mask.cols / 2;
    const int x0 = left ? 0 : split;
    const int x1 = left ? split : white_mask.cols;
    const cv::Mat branch = white_mask(cv::Range(y0, y1), cv::Range(x0, x1));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(branch.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const double minimum = branch.total() * c_.fork_target_min_area_ratio;
    double weighted_x = 0.0;
    double area_sum = 0.0;
    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < minimum) continue;
      const cv::Moments moments = cv::moments(contour);
      if (moments.m00 <= 0.0) continue;
      weighted_x += (moments.m10 / moments.m00 + x0) * area;
      area_sum += area;
    }
    if (area_sum <= 0.0) return std::nullopt;
    const double full_x = roi_x + weighted_x / area_sum;
    return clamp((frame_width / 2.0 - full_x) / (frame_width / 2.0), -1.0, 1.0);
  }

  std::optional<double> hough(
    const cv::Mat & lane_mask, double vehicle_x, double curvature_hint,
    std::optional<double> previous_center, ForkDirection fork_direction,
    bool * both_lanes, bool * branch_pair_selected,
    bool * branch_scene_ambiguous, HoughDebug * debug)
  {
    if (both_lanes) *both_lanes = false;
    if (branch_pair_selected) *branch_pair_selected = false;
    if (branch_scene_ambiguous) *branch_scene_ambiguous = false;
    cv::Mat blur, edges;
    cv::GaussianBlur(lane_mask, blur, cv::Size(5, 5), 0.0);
    cv::Canny(blur, edges, c_.canny_low, c_.canny_high);
    const double base_top = c_.hough_top;
    const double curve_top = std::max(base_top, c_.hough_curve_top);
    const double adaptive_top = base_top +
      (curve_top - base_top) * clamp(curvature_hint, 0.0, 1.0);
    const int top = std::clamp(
      static_cast<int>(clamp(adaptive_top, 0.0, 0.90) * edges.rows), 0, edges.rows - 1);
    const int bottom = edges.rows - 1;
    edges.rowRange(0, top).setTo(0);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1.0, CV_PI / 180.0, c_.hough_threshold,
      c_.hough_min_length, c_.hough_max_gap);

    struct Candidate {
      cv::Vec4i segment;
      double reference_x;
    };
    std::vector<Candidate> left, right, all;
    for (const auto & line : lines) {
      const double dx = line[2] - line[0];
      const double dy = line[3] - line[1];
      // Express the old |dy/dx| threshold without rejecting vertical lanes.
      if (std::abs(dy) < 1e-6 || std::abs(dy) < c_.hough_slope_min * std::abs(dx)) continue;
      const double reference_x = line[0] + (bottom - line[1]) * dx / dy;
      if (!std::isfinite(reference_x) || reference_x < -0.25 * lane_mask.cols ||
        reference_x > 1.25 * lane_mask.cols)
      {
        continue;
      }
      const Candidate candidate{line, reference_x};
      all.push_back(candidate);
      if (reference_x < vehicle_x) {
        left.push_back(candidate);
      } else if (reference_x > vehicle_x) {
        right.push_back(candidate);
      }
    }
    if (debug) {
      debug->segments = lines;
      debug->selected_segments.clear();
      debug->left_curve.reset();
      debug->right_curve.reset();
      debug->edges = edges.clone();
      debug->top_y = top;
    }
    if (left.empty() && right.empty()) return std::nullopt;

    auto fit_curve = [top, bottom](const std::vector<Candidate> & candidates)
      -> std::optional<cv::Vec3d>
    {
      std::vector<cv::Point2d> points;
      constexpr int samples_per_segment = 8;
      points.reserve(candidates.size() * samples_per_segment);
      for (const auto & candidate : candidates) {
        const auto & s = candidate.segment;
        for (int i = 0; i < samples_per_segment; ++i) {
          const double ratio = static_cast<double>(i) / (samples_per_segment - 1);
          points.emplace_back(
            s[0] + ratio * (s[2] - s[0]),
            s[1] + ratio * (s[3] - s[1]));
        }
      }
      if (points.size() < 2) return std::nullopt;

      std::vector<double> ys;
      ys.reserve(points.size());
      for (const auto & point : points) ys.push_back(point.y);
      std::sort(ys.begin(), ys.end());
      int distinct_y = 1;
      for (std::size_t i = 1; i < ys.size(); ++i) {
        if (std::abs(ys[i] - ys[i - 1]) > 1.0) ++distinct_y;
      }
      const bool quadratic = distinct_y >= 3 &&
        ys.back() - ys.front() >= 0.15 * std::max(1, bottom - top);

      auto solve_points = [top, bottom](
        const std::vector<cv::Point2d> & samples, bool use_quadratic)
        -> std::optional<cv::Vec3d>
      {
        const int columns = use_quadratic ? 3 : 2;
        cv::Mat design(static_cast<int>(samples.size()), columns, CV_64F);
        cv::Mat values(static_cast<int>(samples.size()), 1, CV_64F);
        const double span = std::max(1, bottom - top);
        for (std::size_t i = 0; i < samples.size(); ++i) {
          const double t = (samples[i].y - top) / span;
          if (use_quadratic) {
            design.at<double>(static_cast<int>(i), 0) = t * t;
            design.at<double>(static_cast<int>(i), 1) = t;
            design.at<double>(static_cast<int>(i), 2) = 1.0;
          } else {
            design.at<double>(static_cast<int>(i), 0) = t;
            design.at<double>(static_cast<int>(i), 1) = 1.0;
          }
          values.at<double>(static_cast<int>(i), 0) = samples[i].x;
        }
        cv::Mat solution;
        if (!cv::solve(design, values, solution, cv::DECOMP_SVD)) return std::nullopt;
        if (use_quadratic) {
          return cv::Vec3d(
            solution.at<double>(0), solution.at<double>(1), solution.at<double>(2));
        }
        return cv::Vec3d(0.0, solution.at<double>(0), solution.at<double>(1));
      };

      return solve_points(points, quadratic);
    };

    const auto target = [top, bottom](const cv::Vec3d & curve) {
      return evaluate_curve(curve, top, top, bottom);
    };
    const double cluster_limit = std::max(16.0, lane_mask.cols * 0.12);
    auto select_curve = [&](const std::vector<Candidate> & candidates, bool is_left)
      -> std::optional<cv::Vec3d>
    {
      if (candidates.empty()) return std::nullopt;
      const auto seed = is_left ?
        std::max_element(candidates.begin(), candidates.end(),
          [](const auto & a, const auto & b) { return a.reference_x < b.reference_x; }) :
        std::min_element(candidates.begin(), candidates.end(),
          [](const auto & a, const auto & b) { return a.reference_x < b.reference_x; });
      std::vector<Candidate> selected;
      for (const auto & candidate : candidates) {
        if (std::abs(candidate.reference_x - seed->reference_x) <= cluster_limit) {
          selected.push_back(candidate);
          if (debug) debug->selected_segments.push_back(candidate.segment);
        }
      }
      return fit_curve(selected);
    };

    auto left_curve = select_curve(left, true);
    auto right_curve = select_curve(right, false);

    // At the island/fork, both boundaries of the chosen corridor can lie on
    // the same side of the camera center. The normal nearest-left +
    // nearest-right pairing then combines an island edge with the opposite
    // outer edge and steers into the middle. Cluster every visible boundary,
    // form adjacent lane-width pairs, and choose the outermost valid corridor
    // requested by the sign.
    if (fork_direction != ForkDirection::NONE && all.size() >= 2U) {
      std::sort(all.begin(), all.end(), [](const Candidate & a, const Candidate & b) {
        return a.reference_x < b.reference_x;
      });
      std::vector<std::vector<Candidate>> clusters;
      std::vector<double> cluster_means;
      for (const auto & candidate : all) {
        if (clusters.empty() ||
          std::abs(candidate.reference_x - cluster_means.back()) > cluster_limit)
        {
          clusters.push_back({candidate});
          cluster_means.push_back(candidate.reference_x);
        } else {
          const double count = static_cast<double>(clusters.back().size());
          cluster_means.back() =
            (cluster_means.back() * count + candidate.reference_x) / (count + 1.0);
          clusters.back().push_back(candidate);
        }
      }

      struct BoundaryFit {
        cv::Vec3d curve;
        double reference_x;
        double target_x;
        std::vector<Candidate> candidates;
      };
      std::vector<BoundaryFit> boundaries;
      for (std::size_t i = 0; i < clusters.size(); ++i) {
        const auto curve = fit_curve(clusters[i]);
        if (!curve) continue;
        boundaries.push_back({*curve, cluster_means[i], target(*curve), clusters[i]});
      }

      std::optional<std::size_t> selected_pair;
      int valid_pair_count = 0;
      const double minimum_width = lane_mask.cols * c_.lane_width_min;
      const double maximum_width = lane_mask.cols * c_.lane_width_max;
      for (std::size_t i = 0; i + 1 < boundaries.size(); ++i) {
        const double width = boundaries[i + 1].target_x - boundaries[i].target_x;
        if (width < minimum_width || width > maximum_width) continue;
        ++valid_pair_count;
        if (!selected_pair || fork_direction == ForkDirection::RIGHT) {
          selected_pair = i;
        }
      }
      if (branch_scene_ambiguous) {
        *branch_scene_ambiguous = valid_pair_count >= 2;
      }
      if (selected_pair) {
        const auto & outer = boundaries[*selected_pair];
        const auto & inner = boundaries[*selected_pair + 1];
        left_curve = outer.curve;
        right_curve = inner.curve;
        if (branch_pair_selected) *branch_pair_selected = true;
        if (debug) {
          debug->selected_segments.clear();
          for (const auto & candidate : outer.candidates) {
            debug->selected_segments.push_back(candidate.segment);
          }
          for (const auto & candidate : inner.candidates) {
            debug->selected_segments.push_back(candidate.segment);
          }
        }
      }
    }
    if (debug) {
      debug->left_curve = left_curve;
      debug->right_curve = right_curve;
    }
    if (!left_curve && !right_curve) return std::nullopt;
    const std::optional<double> left_target = left_curve ?
      std::optional<double>(target(*left_curve)) : std::nullopt;
    const std::optional<double> right_target = right_curve ?
      std::optional<double>(target(*right_curve)) : std::nullopt;
    double lane_width = tracked_lane_width_.value_or(
      lane_mask.cols * c_.assumed_lane_width);
    if (left_target && right_target) {
      const double observed_width = *right_target - *left_target;
      const double minimum = lane_mask.cols * c_.lane_width_min;
      const double maximum = lane_mask.cols * c_.lane_width_max;
      if (observed_width >= minimum && observed_width <= maximum) {
        if (both_lanes) *both_lanes = true;
        if (tracked_lane_width_) {
          *tracked_lane_width_ += clamp(c_.lane_width_smoothing, 0.0, 1.0) *
            (observed_width - *tracked_lane_width_);
        } else {
          tracked_lane_width_ = observed_width;
        }
        lane_width = *tracked_lane_width_;
      }
      previous_left_target_ = *left_target;
      previous_right_target_ = *right_target;
      previous_single_is_left_.reset();
      return (*left_target + *right_target) / 2.0;
    }

    // A sole boundary may cross the camera center in a hairpin.  Evaluate both
    // left/right center hypotheses against the previous boundary tracks and
    // previous lane center; the image half is only the no-history fallback.
    const double detected = left_target.value_or(*right_target);
    const bool raw_is_left = left_target.has_value();
    const double left_center = detected + lane_width / 2.0;
    const double right_center = detected - lane_width / 2.0;
    auto identity_score = [&](bool is_left) -> std::optional<double> {
      double score = 0.0;
      int evidence = 0;
      const auto & boundary = is_left ? previous_left_target_ : previous_right_target_;
      if (boundary) { score += std::abs(detected - *boundary); ++evidence; }
      if (previous_center) {
        score += std::abs((is_left ? left_center : right_center) - *previous_center);
        ++evidence;
      }
      if (evidence == 0) return std::nullopt;
      return score / evidence;
    };
    const auto left_score = identity_score(true);
    const auto right_score = identity_score(false);
    bool is_left = raw_is_left;
    if (left_score && right_score) {
      is_left = *left_score <= *right_score;
      const double margin = lane_mask.cols * c_.single_lane_switch_margin;
      if (previous_single_is_left_ && is_left != *previous_single_is_left_) {
        const double previous_score = *previous_single_is_left_ ? *left_score : *right_score;
        const double alternative_score = *previous_single_is_left_ ? *right_score : *left_score;
        if (previous_score <= alternative_score + margin) {
          is_left = *previous_single_is_left_;
        }
      }
    }
    if (debug && is_left != raw_is_left) {
      const auto moved_curve = raw_is_left ? debug->left_curve : debug->right_curve;
      if (is_left) {
        debug->left_curve = moved_curve;
        debug->right_curve.reset();
      } else {
        debug->right_curve = moved_curve;
        debug->left_curve.reset();
      }
    }
    previous_single_is_left_ = is_left;
    if (is_left) previous_left_target_ = detected;
    else previous_right_target_ = detected;
    return is_left ? left_center : right_center;
  }

  Config c_;
  cv::Ptr<cv::CLAHE> clahe_;
  cv::Mat open_kernel_, close_kernel_;
  std::optional<double> previous_center_;
  std::optional<double> previous_left_target_, previous_right_target_;
  std::optional<double> tracked_lane_width_;
  std::optional<bool> previous_single_is_left_;
  double filtered_curvature_{0.0};
  double previous_error_{0.0};
};

class Controller {
public:
  explicit Controller(const Config & c) : c_(c) {}

  void update_config(const Config & config) { c_ = config; }

  Command follow(
    const LaneObs & lane, double cap, double steer_limit,
    std::optional<double> floor = std::nullopt, double curve_scale = 1.0)
  {
    if (!lane.valid) {
      previous_steer_ = rate(previous_steer_ * c_.lost_decay, previous_steer_, c_.steer_rate);
      previous_throttle_ = std::min(previous_throttle_, c_.speed_min);
      return {previous_throttle_, previous_steer_};
    }
    const double steer = steering(lane, steer_limit, curve_scale);
    return {throttle(cap, steer, lane.curvature, floor), steer};
  }

  Command follow_with_startup(const LaneObs & lane, double cap, double steer_limit) {
    if (lane.valid) return follow(lane, cap, steer_limit);

    // The start grid can contain no usable OUT white line.  A normal lost-lane
    // command preserves the previous throttle, which is zero while waiting for
    // green and therefore deadlocks AUTO at the start.  Launch straight at the
    // configured rolling minimum until white paint becomes visible; steering
    // still decays toward center instead of following the excluded yellow line.
    previous_steer_ = rate(previous_steer_ * c_.lost_decay, previous_steer_, c_.steer_rate);
    const double section_cap = clamp(cap, c_.speed_min, c_.speed_max);
    previous_throttle_ = clamp(
      std::max(previous_throttle_, c_.speed_min), c_.speed_min, section_cap);
    return {previous_throttle_, previous_steer_};
  }

  Command directional_fork(
    const LaneObs & lane, const std::string & decision,
    double cap, double steer_limit)
  {
    LaneObs virtual_lane = lane;
    // Start moving toward the signed side target as soon as the sign vote is
    // locked.  Do not wait for, or steer at, a contour in the upper arm of the
    // X: at that point the car has already spent too long driving straight.
    virtual_lane.valid = true;
    if (decision == "LEFT") {
      virtual_lane.center_error = c_.fork_forced_error;
    } else if (decision == "RIGHT") {
      virtual_lane.center_error = -c_.fork_forced_error;
    }
    const double steer = steering(virtual_lane, steer_limit, c_.fork_curve_scale);
    return {throttle(cap, steer, lane.curvature), steer};
  }

  void stop() { previous_throttle_ = 0.0; }

private:
  static double rate(double target, double previous, double delta) {
    return previous + clamp(target - previous, -delta, delta);
  }
  double steering(const LaneObs & lane, double limit, double curve_scale = 1.0) {
    curve_scale = clamp(curve_scale, 0.0, 1.0);
    const double target = clamp(
      lane.center_error + c_.curve_blend * curve_scale * lane.signed_curvature, -1.0, 1.0);
    const double curve_strength = std::pow(
      clamp(lane.curvature * curve_scale, 0.0, 1.0),
      std::max(c_.curve_response_power, 1e-3));
    const double minimum = std::min(c_.lookahead, c_.curve_lookahead_min);
    const double lookahead = std::max(
      c_.lookahead + curve_strength * (minimum - c_.lookahead), 1e-3);
    const double alpha = std::atan2(target * c_.lateral_scale, lookahead);
    const double delta = std::atan2(2.0 * c_.wheelbase * std::sin(alpha), lookahead);
    const double curve_gain = 1.0 + std::max(c_.curve_steer_boost, 0.0) * curve_strength;
    double raw = c_.pp_gain * curve_gain * delta /
      (std::max(c_.max_steer_deg, 1.0) * CV_PI / 180.0);
    raw = clamp(raw, -limit, limit);
    previous_steer_ = rate(raw, previous_steer_, c_.steer_rate);
    return c_.steer_sign * previous_steer_;
  }
  double throttle(double cap, double steer, double curvature, std::optional<double> requested_floor = std::nullopt) {
    cap = clamp(cap, c_.speed_min, c_.speed_max);
    const double floor = std::min(cap, requested_floor.value_or(c_.speed_min));
    const double steer_demand = std::max(std::abs(steer) - c_.straight_steer_deadband, 0.0);
    const double curvature_demand = std::max(
      std::abs(curvature) - c_.straight_curvature_deadband, 0.0);
    double target = clamp(
      cap - c_.steer_slowdown * steer_demand - c_.curvature_slowdown * curvature_demand,
      floor, cap);
    if (target > previous_throttle_) {
      target = previous_throttle_ <= 0.0 ? floor :
        std::min(target, previous_throttle_ + c_.ramp_up);
    }
    target = clamp(target, floor, cap);
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
    const std::string route_mode_text = declare_parameter<std::string>("route_mode", "OUT");
    route_mode_ = parse_route_mode(route_mode_text);
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image/compressed");
    control_topic_ = declare_parameter<std::string>("control_topic", "/control");
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/bisa/detections");
    mission_state_topic_ = declare_parameter<std::string>(
      "mission_state_topic", "/bisa/mission_state");
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
    state_pub_ = create_publisher<std_msgs::msg::String>(mission_state_topic_, 1);
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
    state_ = route_mode_ == RouteMode::LANE ? MissionState::LANE_TEST :
      route_mode_ == RouteMode::IN ? MissionState::IN_WAIT_GREEN :
      MissionState::OUT_WAIT_GREEN;
    has_started_ = route_mode_ == RouteMode::LANE;
    performance_started_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(),
      "C++ BISA core started: route=%s state=%s perception=%.1f Hz control=%.1f Hz debug=%.1f Hz",
      route_mode_text.c_str(), state_name(state_), perception_hz_, control_hz_, debug_hz_);
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
    config_.out_white_only = declare_parameter<bool>("lane.out_white_only", config_.out_white_only);
    config_.fork_target_y0 = declare_parameter<double>("lane.fork_target_y0", config_.fork_target_y0);
    config_.fork_target_y1 = declare_parameter<double>("lane.fork_target_y1", config_.fork_target_y1);
    config_.fork_target_min_area_ratio = declare_parameter<double>(
      "lane.fork_target_min_area_ratio", config_.fork_target_min_area_ratio);
    config_.clahe_clip = declare_parameter<double>("lane.lab_clahe_clip", config_.clahe_clip);
    config_.clahe_tile = declare_parameter<int>("lane.lab_clahe_tile", config_.clahe_tile);
    config_.morph_open = declare_parameter<int>("lane.morph_open_kernel", config_.morph_open);
    config_.morph_close = declare_parameter<int>("lane.morph_close_kernel", config_.morph_close);
    config_.min_area_ratio = declare_parameter<double>(
      "lane.min_component_area_ratio", config_.min_area_ratio);
    config_.fork_area_ratio = declare_parameter<double>("lane.fork_area_ratio", config_.fork_area_ratio);
    config_.hough_top = declare_parameter<double>("lane.hough_roi_top_ratio", config_.hough_top);
    config_.hough_curve_top = declare_parameter<double>(
      "lane.hough_curve_top_ratio", config_.hough_curve_top);
    config_.hough_curvature_smoothing = declare_parameter<double>(
      "lane.hough_curvature_smoothing", config_.hough_curvature_smoothing);
    config_.canny_low = declare_parameter<int>("lane.hough_canny_low", config_.canny_low);
    config_.canny_high = declare_parameter<int>("lane.hough_canny_high", config_.canny_high);
    config_.hough_threshold = declare_parameter<int>("lane.hough_threshold", config_.hough_threshold);
    config_.hough_min_length = declare_parameter<int>(
      "lane.hough_min_line_length", config_.hough_min_length);
    config_.hough_max_gap = declare_parameter<int>("lane.hough_max_line_gap", config_.hough_max_gap);
    config_.hough_slope_min = declare_parameter<double>(
      "lane.hough_slope_min_abs", config_.hough_slope_min);
    config_.assumed_lane_width = declare_parameter<double>(
      "lane.assumed_lane_width_ratio", config_.assumed_lane_width);
    config_.lane_width_min = declare_parameter<double>(
      "lane.lane_width_min_ratio", config_.lane_width_min);
    config_.lane_width_max = declare_parameter<double>(
      "lane.lane_width_max_ratio", config_.lane_width_max);
    config_.lane_width_smoothing = declare_parameter<double>(
      "lane.lane_width_smoothing", config_.lane_width_smoothing);
    config_.single_lane_switch_margin = declare_parameter<double>(
      "lane.single_lane_switch_margin_ratio", config_.single_lane_switch_margin);
    config_.max_center_jump = declare_parameter<double>(
      "lane.max_center_jump", config_.max_center_jump);

    config_.lookahead = declare_parameter<double>("steering.lookahead_m", config_.lookahead);
    config_.curve_lookahead_min = declare_parameter<double>(
      "steering.curve_lookahead_min_m", config_.curve_lookahead_min);
    config_.curve_response_power = declare_parameter<double>(
      "steering.curve_response_power", config_.curve_response_power);
    config_.curve_steer_boost = declare_parameter<double>(
      "steering.curve_steer_boost", config_.curve_steer_boost);
    config_.fork_curve_scale = declare_parameter<double>(
      "steering.fork_curve_scale", config_.fork_curve_scale);
    config_.fork_forced_error = declare_parameter<double>(
      "steering.fork_forced_error", config_.fork_forced_error);
    config_.wheelbase = declare_parameter<double>("steering.wheelbase_m", config_.wheelbase);
    config_.lateral_scale = declare_parameter<double>("steering.lateral_scale_m", config_.lateral_scale);
    config_.max_steer_deg = declare_parameter<double>("steering.max_steer_deg", config_.max_steer_deg);
    config_.pp_gain = declare_parameter<double>("steering.pp_gain", config_.pp_gain);
    config_.curve_blend = declare_parameter<double>("steering.curve_blend", config_.curve_blend);
    config_.straight_limit = declare_parameter<double>("steering.straight_limit", config_.straight_limit);
    config_.s_curve_limit = declare_parameter<double>("steering.s_curve_limit", config_.s_curve_limit);
    config_.fork_approach_limit = declare_parameter<double>(
      "steering.fork_approach_limit", config_.fork_approach_limit);
    config_.fork_limit = declare_parameter<double>("steering.fork_limit", config_.fork_limit);
    config_.post_fork_limit = declare_parameter<double>(
      "steering.post_fork_limit", config_.post_fork_limit);
    config_.steer_rate = declare_parameter<double>("steering.rate_limit_per_cmd", config_.steer_rate);
    config_.steer_sign = declare_parameter<int>("steering.steer_sign", config_.steer_sign);

    config_.speed_min = declare_parameter<double>("throttle.speed_min", config_.speed_min);
    config_.speed_max = declare_parameter<double>("throttle.speed_max", config_.speed_max);
    config_.launch_cap = declare_parameter<double>("throttle.launch_cap", config_.launch_cap);
    config_.s_curve_cap = declare_parameter<double>("throttle.s_curve_cap", config_.s_curve_cap);
    config_.fork_approach_cap = declare_parameter<double>(
      "throttle.fork_approach_cap", config_.fork_approach_cap);
    config_.fork_commit_cap = declare_parameter<double>(
      "throttle.fork_commit_cap", config_.fork_commit_cap);
    config_.post_fork_cap = declare_parameter<double>(
      "throttle.post_fork_cap", config_.post_fork_cap);
    config_.straight_steer_deadband = declare_parameter<double>(
      "throttle.straight_steer_deadband", config_.straight_steer_deadband);
    config_.straight_curvature_deadband = declare_parameter<double>(
      "throttle.straight_curvature_deadband", config_.straight_curvature_deadband);

    config_.sign_stop_delay_sec = declare_parameter<double>(
      "mission.sign_stop_delay_sec", config_.sign_stop_delay_sec);
    config_.fork_sign_advance_sec = declare_parameter<double>(
      "mission.fork_sign_advance_sec", config_.fork_sign_advance_sec);
    config_.fork_commit_min_sec = declare_parameter<double>(
      "mission.fork_commit_min_sec", config_.fork_commit_min_sec);
    config_.fork_commit_timeout_sec = declare_parameter<double>(
      "mission.fork_commit_timeout_sec", config_.fork_commit_timeout_sec);
    config_.aruco_confirm_frames = declare_parameter<int>(
      "aruco.confirm_frames", config_.aruco_confirm_frames);
    config_.aruco_clear_frames = declare_parameter<int>(
      "aruco.clear_frames", config_.aruco_clear_frames);

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
        else if (name == "lane.out_white_only") next.out_white_only = parameter.as_bool();
        else if (name == "lane.fork_target_y0") next.fork_target_y0 = parameter.as_double();
        else if (name == "lane.fork_target_y1") next.fork_target_y1 = parameter.as_double();
        else if (name == "lane.fork_target_min_area_ratio") {
          next.fork_target_min_area_ratio = parameter.as_double();
        }
        else if (name == "lane.lab_clahe_clip") next.clahe_clip = parameter.as_double();
        else if (name == "lane.lab_clahe_tile") next.clahe_tile = parameter.as_int();
        else if (name == "lane.morph_open_kernel") next.morph_open = parameter.as_int();
        else if (name == "lane.morph_close_kernel") next.morph_close = parameter.as_int();
        else if (name == "lane.min_component_area_ratio") next.min_area_ratio = parameter.as_double();
        else if (name == "lane.fork_area_ratio") next.fork_area_ratio = parameter.as_double();
        else if (name == "lane.hough_roi_top_ratio") next.hough_top = parameter.as_double();
        else if (name == "lane.hough_curve_top_ratio") next.hough_curve_top = parameter.as_double();
        else if (name == "lane.hough_curvature_smoothing") {
          next.hough_curvature_smoothing = parameter.as_double();
        }
        else if (name == "lane.hough_canny_low") next.canny_low = parameter.as_int();
        else if (name == "lane.hough_canny_high") next.canny_high = parameter.as_int();
        else if (name == "lane.hough_threshold") next.hough_threshold = parameter.as_int();
        else if (name == "lane.hough_min_line_length") next.hough_min_length = parameter.as_int();
        else if (name == "lane.hough_max_line_gap") next.hough_max_gap = parameter.as_int();
        else if (name == "lane.hough_slope_min_abs") next.hough_slope_min = parameter.as_double();
        else if (name == "lane.assumed_lane_width_ratio") next.assumed_lane_width = parameter.as_double();
        else if (name == "lane.lane_width_min_ratio") next.lane_width_min = parameter.as_double();
        else if (name == "lane.lane_width_max_ratio") next.lane_width_max = parameter.as_double();
        else if (name == "lane.lane_width_smoothing") next.lane_width_smoothing = parameter.as_double();
        else if (name == "lane.single_lane_switch_margin_ratio") {
          next.single_lane_switch_margin = parameter.as_double();
        }
        else if (name == "lane.max_center_jump") next.max_center_jump = parameter.as_double();
        else if (name == "steering.lookahead_m") next.lookahead = parameter.as_double();
        else if (name == "steering.curve_lookahead_min_m") {
          next.curve_lookahead_min = parameter.as_double();
        }
        else if (name == "steering.curve_response_power") {
          next.curve_response_power = parameter.as_double();
        }
        else if (name == "steering.curve_steer_boost") {
          next.curve_steer_boost = parameter.as_double();
        }
        else if (name == "steering.fork_curve_scale") {
          next.fork_curve_scale = parameter.as_double();
        }
        else if (name == "steering.fork_forced_error") {
          next.fork_forced_error = parameter.as_double();
        }
        else if (name == "steering.wheelbase_m") next.wheelbase = parameter.as_double();
        else if (name == "steering.lateral_scale_m") next.lateral_scale = parameter.as_double();
        else if (name == "steering.max_steer_deg") next.max_steer_deg = parameter.as_double();
        else if (name == "steering.pp_gain") next.pp_gain = parameter.as_double();
        else if (name == "steering.curve_blend") next.curve_blend = parameter.as_double();
        else if (name == "steering.straight_limit") next.straight_limit = parameter.as_double();
        else if (name == "steering.s_curve_limit") next.s_curve_limit = parameter.as_double();
        else if (name == "steering.fork_approach_limit") {
          next.fork_approach_limit = parameter.as_double();
        }
        else if (name == "steering.fork_limit") next.fork_limit = parameter.as_double();
        else if (name == "steering.post_fork_limit") next.post_fork_limit = parameter.as_double();
        else if (name == "steering.rate_limit_per_cmd") next.steer_rate = parameter.as_double();
        else if (name == "steering.steer_sign") next.steer_sign = parameter.as_int();
        else if (name == "throttle.speed_min") next.speed_min = parameter.as_double();
        else if (name == "throttle.speed_max") next.speed_max = parameter.as_double();
        else if (name == "throttle.launch_cap") next.launch_cap = parameter.as_double();
        else if (name == "throttle.s_curve_cap") next.s_curve_cap = parameter.as_double();
        else if (name == "throttle.fork_approach_cap") {
          next.fork_approach_cap = parameter.as_double();
        }
        else if (name == "throttle.fork_commit_cap") {
          next.fork_commit_cap = parameter.as_double();
        }
        else if (name == "throttle.post_fork_cap") {
          next.post_fork_cap = parameter.as_double();
        }
        else if (name == "throttle.straight_steer_deadband") {
          next.straight_steer_deadband = parameter.as_double();
        }
        else if (name == "throttle.straight_curvature_deadband") {
          next.straight_curvature_deadband = parameter.as_double();
        }
        else if (name == "mission.sign_stop_delay_sec") {
          next.sign_stop_delay_sec = parameter.as_double();
        }
        else if (name == "mission.fork_sign_advance_sec") {
          next.fork_sign_advance_sec = parameter.as_double();
        }
        else if (name == "mission.fork_commit_min_sec") {
          next.fork_commit_min_sec = parameter.as_double();
        }
        else if (name == "mission.fork_commit_timeout_sec") {
          next.fork_commit_timeout_sec = parameter.as_double();
        }
        else if (name == "aruco.confirm_frames") {
          next.aruco_confirm_frames = parameter.as_int();
        }
        else if (name == "aruco.clear_frames") next.aruco_clear_frames = parameter.as_int();
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
      next.curve_lookahead_min > 0.0 && next.curve_lookahead_min <= next.lookahead &&
      next.curve_response_power > 0.0 && next.curve_steer_boost >= 0.0 &&
      next.fork_curve_scale >= 0.0 && next.fork_curve_scale <= 1.0 &&
      next.max_steer_deg > 0.0 && next.gamma > 0.0 &&
      next.hough_top >= 0.0 && next.hough_top < 0.90 &&
      next.hough_curve_top >= next.hough_top && next.hough_curve_top <= 0.90 &&
      next.hough_curvature_smoothing >= 0.0 && next.hough_curvature_smoothing <= 1.0 &&
      next.fork_target_y0 >= 0.0 && next.fork_target_y0 < next.fork_target_y1 &&
      next.fork_target_y1 <= 1.0 && next.fork_target_min_area_ratio >= 0.0 &&
      next.lane_width_min > 0.0 && next.lane_width_min < next.lane_width_max &&
      next.lane_width_max <= 1.0 && next.lane_width_smoothing >= 0.0 &&
      next.lane_width_smoothing <= 1.0 && next.single_lane_switch_margin >= 0.0;
    const auto valid_cap = [&next](double cap) {
      return cap >= next.speed_min && cap <= next.speed_max;
    };
    const bool speed_valid = next.speed_min >= 0.0 && next.speed_min <= next.speed_max &&
      next.speed_max <= 1.0 && valid_cap(next.launch_cap) && valid_cap(next.s_curve_cap) &&
      valid_cap(next.fork_approach_cap) && valid_cap(next.fork_commit_cap) &&
      valid_cap(next.post_fork_cap) && next.straight_steer_deadband >= 0.0 &&
      next.straight_steer_deadband <= 1.0 && next.straight_curvature_deadband >= 0.0 &&
      next.straight_curvature_deadband <= 1.0 && next.fork_forced_error >= 0.0 &&
      next.fork_forced_error <= 1.0 &&
      next.sign_stop_delay_sec >= 0.0 && next.fork_sign_advance_sec >= 0.0 &&
      next.fork_commit_min_sec >= 0.0 &&
      next.fork_commit_timeout_sec >= next.fork_commit_min_sec &&
      next.aruco_confirm_frames >= 1 && next.aruco_clear_frames >= 1;
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
    bool include_yellow = true;
    ForkDirection fork_direction = ForkDirection::NONE;
    int aruco_target_id = 3;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      include_yellow = route_mode_ != RouteMode::OUT || !config_.out_white_only;
      aruco_target_id = config_.aruco_target_id;
      if (state_ == MissionState::OUT_FORK_SIGN_ADVANCE ||
        state_ == MissionState::OUT_FORK_COMMIT)
      {
        fork_direction = fork_decision_ == "LEFT" ? ForkDirection::LEFT :
          fork_decision_ == "RIGHT" ? ForkDirection::RIGHT : ForkDirection::NONE;
      }
    }
    {
      std::lock_guard<std::mutex> lane_lock(lane_mutex_);
      if (lane_reset_requested_.exchange(false)) lane_.reset_fork_history();
      observation = lane_.process(
        frame, include_yellow, fork_direction,
        publish_debug_ ? &lane_debug : nullptr);
    }
    bool marker = false;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50), corners, ids);
    marker = std::find(ids.begin(), ids.end(), aruco_target_id) != ids.end();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_lane_ = observation;
      target_marker_ = marker;
      if (!red_stop_armed_) {
        aruco_arm_streak_ = marker ?
          std::min(aruco_arm_streak_ + 1, config_.aruco_confirm_frames) : 0;
        if (aruco_arm_streak_ >= config_.aruco_confirm_frames) {
          red_stop_armed_ = true;
          RCLCPP_INFO(
            get_logger(), "red-light stop armed after target ArUco id=%d",
            config_.aruco_target_id);
        }
      }
      marker_ids_ = std::move(ids);
      marker_corners_ = std::move(corners);
      if (publish_debug_) {
        latest_frame_ = frame;
        latest_lane_debug_ = std::move(lane_debug);
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
    const int light = static_cast<int>(msg->data[3]);
    const int count = std::max(0, static_cast<int>(msg->data[4]));
    std::vector<Detection> detections;
    std::array<double, 4> light_roi{0.00, 0.00, 0.80, 0.85};
    bool left = false, right = false;
    for (int i = 0; i < count; ++i) {
      const std::size_t base = 5 + static_cast<std::size_t>(i) * 6;
      if (base + 5 >= msg->data.size()) break;
      Detection d;
      d.class_id = static_cast<int>(msg->data[base]);
      d.confidence = msg->data[base + 1];
      const float x1 = msg->data[base + 2], y1 = msg->data[base + 3];
      const float x2 = msg->data[base + 4], y2 = msg->data[base + 5];
      if (!std::isfinite(d.confidence) || !std::isfinite(x1) || !std::isfinite(y1) ||
        !std::isfinite(x2) || !std::isfinite(y2) || x2 <= x1 || y2 <= y1)
      {
        continue;
      }
      d.box = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
      left = left || d.class_id == 2; right = right || d.class_id == 3;
      detections.push_back(d);
    }
    const std::size_t roi_base = 5 + static_cast<std::size_t>(count) * 6;
    if (roi_base + 3 < msg->data.size()) {
      std::array<double, 4> candidate{
        msg->data[roi_base], msg->data[roi_base + 1],
        msg->data[roi_base + 2], msg->data[roi_base + 3]};
      if (std::all_of(candidate.begin(), candidate.end(), [](double value) {
          return std::isfinite(value) && value >= 0.0 && value <= 1.0;
        }) && candidate[0] < candidate[2] && candidate[1] < candidate[3])
      {
        light_roi = candidate;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    detection_sequence_ = sequence;
    light_state_ = light;
    light_received_ = now();
    detections_ = detections;
    light_roi_ = light_roi;
    red_streak_ = has_started_ && red_stop_armed_ && light == 2 ?
      std::min(red_streak_ + 1, kRedConfirmFrames) : 0;
    if (should_latch_red(
        has_started_, red_stop_armed_, light, red_streak_, kRedConfirmFrames) &&
      !red_stop_latched_.exchange(true))
    {
      controller_.stop();
      control_msgs::msg::Control stop;
      stop.header.stamp = now();
      stop.steering = 0.0F;
      stop.throttle = 0.0F;
      {
        std::lock_guard<std::mutex> output_lock(control_output_mutex_);
        control_pub_->publish(stop);
      }
      RCLCPP_WARN(
        get_logger(),
        "global red-light stop latched after ArUco gate and %d consecutive detections",
        red_streak_);
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
    if (fresh == 0) { green_streak_ = 0; return; }
    if (detection_sequence_ == last_light_sequence_) return;
    last_light_sequence_ = detection_sequence_;
    green_streak_ = fresh == 1 ? green_streak_ + 1 : 0;
    (void)now_sec;
  }

  void transition(MissionState next, double now_sec) {
    if (state_ == next) return;
    RCLCPP_INFO(get_logger(), "mission state: %s -> %s", state_name(state_), state_name(next));
    if (next == MissionState::OUT_FORK_SIGN_ADVANCE) {
      fork_scene_observed_ = false;
      fork_pair_streak_ = 0;
      fork_clear_streak_ = 0;
    }
    if (next == MissionState::OUT_FORK_COMMIT) {
      fork_pair_streak_ = 0;
      fork_clear_streak_ = 0;
    }
    state_ = next;
    entered_ = now_sec;
  }

  Command step_out(const LaneObs & lane, double now_sec) {
    switch (state_) {
      case MissionState::OUT_WAIT_GREEN:
        if (green_streak_ >= config_.light_confirm_frames) {
          transition(MissionState::OUT_TO_FORK, now_sec);
        }
        return {};

      case MissionState::OUT_TO_FORK: {
        // S bends are ordinary lane geometry, not a timed mission state. Run
        // the whole green-to-sign section at the global speed cap and let
        // steering demand + measured curvature continuously pull it toward
        // speed_min. OUT perception is white-only, so an adjacent yellow IN
        // split cannot influence center, curvature, or steering.
        auto cmd = controller_.follow_with_startup(
          lane, config_.launch_cap, config_.straight_limit);
        if (sign_decision()) {
          transition(MissionState::OUT_SIGN_APPROACH, now_sec);
        }
        return cmd;
      }

      case MissionState::OUT_SIGN_APPROACH:
        if (now_sec - entered_ >= config_.sign_stop_delay_sec) {
          controller_.stop();
          sign_history_.clear();
          transition(MissionState::OUT_SIGN_VOTE_STOP, now_sec);
          return {};
        }
        return controller_.follow_with_startup(
          lane, config_.launch_cap, config_.straight_limit);

      case MissionState::OUT_SIGN_VOTE_STOP:
        controller_.stop();
        if (auto decision = sign_decision()) {
          fork_decision_ = *decision;
          transition(MissionState::OUT_FORK_SIGN_ADVANCE, now_sec);
        } else if (now_sec - entered_ >= config_.sign_stop_delay_sec) {
          sign_history_.clear();
          transition(MissionState::OUT_TO_FORK, now_sec);
        }
        return {};

      case MissionState::OUT_FORK_SIGN_ADVANCE: {
        // Apply the same signed directional nudge for LEFT and RIGHT.
        fork_scene_observed_ =
          fork_scene_observed_ || lane.branch_scene_ambiguous;
        auto cmd = controller_.directional_fork(
          lane, fork_decision_, config_.fork_approach_cap,
          config_.fork_approach_limit);
        if (now_sec - entered_ >= config_.fork_sign_advance_sec) {
          lane_reset_requested_.store(true);
          transition(MissionState::OUT_FORK_COMMIT, now_sec);
        }
        return cmd;
      }

      case MissionState::OUT_FORK_COMMIT: {
        const double elapsed = now_sec - entered_;
        constexpr int pair_confirm_ticks = 3;
        constexpr int clear_confirm_ticks = 3;
        fork_scene_observed_ =
          fork_scene_observed_ || lane.branch_scene_ambiguous;
        // Only the sign-scoped outer-corridor pair may take steering authority.
        // A generic center pair is never accepted while the island is visible.
        const bool pair_candidate =
          lane.valid && lane.both_lanes && lane.branch_pair_selected;
        fork_pair_streak_ = pair_candidate ?
          std::min(fork_pair_streak_ + 1, pair_confirm_ticks) : 0;
        const bool pair_locked = fork_pair_streak_ >= pair_confirm_ticks;
        auto cmd = pair_locked ?
          controller_.follow(lane, config_.fork_commit_cap, config_.fork_limit) :
          controller_.directional_fork(
            lane, fork_decision_, config_.fork_commit_cap, config_.fork_limit);
        const bool island_cleared =
          fork_scene_observed_ && pair_locked && !lane.branch_scene_ambiguous;
        fork_clear_streak_ = island_cleared ?
          std::min(fork_clear_streak_ + 1, clear_confirm_ticks) : 0;
        const bool reacquired =
          fork_clear_streak_ >= clear_confirm_ticks && std::abs(lane.center_error) < 0.25;
        if ((reacquired && elapsed >= config_.fork_commit_min_sec) ||
          elapsed >= config_.fork_commit_timeout_sec)
        {
          RCLCPP_INFO(
            get_logger(),
            "fork alignment release: scene=%s branch_pair=%s pair_ticks=%d clear_ticks=%d centered=%s timeout=%s",
            fork_scene_observed_ ? "true" : "false",
            lane.branch_pair_selected ? "true" : "false",
            fork_pair_streak_,
            fork_clear_streak_,
            std::abs(lane.center_error) < 0.25 ? "true" : "false",
            elapsed >= config_.fork_commit_timeout_sec ? "true" : "false");
          transition(MissionState::OUT_RESUME, now_sec);
        }
        return cmd;
      }

      case MissionState::OUT_RESUME:
        return controller_.follow(
          lane, config_.post_fork_cap, config_.post_fork_limit, config_.post_fork_min);

      default:
        controller_.stop();
        return {};
    }
  }

  Command step_in(const LaneObs & lane, double now_sec) {
    // IN is deliberately isolated from OUT. Only green -> entry is active in
    // this iteration; dash-line side selection and stop-line lap counting are
    // explicit future gates before IN_ENTRY can transition to IN_LAP/IN_EXIT.
    switch (state_) {
      case MissionState::IN_WAIT_GREEN:
        if (green_streak_ >= config_.light_confirm_frames) {
          transition(MissionState::IN_ENTRY, now_sec);
        }
        return {};
      case MissionState::IN_ENTRY:
      case MissionState::IN_LAP:
        return controller_.follow(lane, config_.s_curve_cap, config_.s_curve_limit);
      case MissionState::IN_EXIT:
      case MissionState::IN_RESUME:
        return controller_.follow(lane, config_.post_fork_cap, config_.post_fork_limit);
      default:
        controller_.stop();
        return {};
    }
  }

  Command step(const LaneObs & lane, double now_sec) {
    if (route_mode_ == RouteMode::LANE) {
      return controller_.follow(lane, config_.speed_max, config_.s_curve_limit);
    }
    return route_mode_ == RouteMode::IN ? step_in(lane, now_sec) : step_out(lane, now_sec);
  }

  void control_loop() {
    LaneObs lane;
    bool marker;
    int light;
    std::vector<int> markers;
    Command cmd;
    double speed_max;
    std::optional<std::string> decision;
    std::string mission_state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double now_sec = now().seconds();
      update_light(now_sec);
      lane = latest_lane_; marker = target_marker_; markers = marker_ids_;
      const double light_age = (now() - light_received_).seconds();
      light = light_age >= 0.0 && light_age <= config_.light_stale_sec ? light_state_ : 0;

      // Safety/mission stops are global control overrides, not FSM states.
      // The marker is intentionally level-triggered: stop on the first camera
      // frame that contains the target and resume on the first frame without
      // it.  Pausing entered_ prevents a timed fork state from expiring while
      // the vehicle is held stationary.
      if (marker && !aruco_stop_active_) {
        aruco_stop_active_ = true;
        aruco_pause_started_ = now_sec;
        RCLCPP_INFO(get_logger(), "global ArUco stop active (target id=%d)", config_.aruco_target_id);
      } else if (!marker && aruco_stop_active_) {
        aruco_stop_active_ = false;
        entered_ += std::max(0.0, now_sec - aruco_pause_started_);
        RCLCPP_INFO(get_logger(), "global ArUco stop cleared; resume mission");
      }
      if (red_stop_latched_.load() || aruco_stop_active_) {
        controller_.stop();
        cmd = {};
      } else {
        cmd = step(lane, now_sec);
        if (!has_started_ && state_ != MissionState::OUT_WAIT_GREEN &&
          state_ != MissionState::IN_WAIT_GREEN)
        {
          has_started_ = true;
        }
      }
      last_command_ = cmd;
      decision = sign_decision();
      mission_state = state_name(state_);
      speed_max = config_.speed_max;
    }
    {
      std::lock_guard<std::mutex> output_lock(control_output_mutex_);
      const bool red_latched = red_stop_latched_.load();
      control_msgs::msg::Control output;
      output.header.stamp = now();
      output.steering = red_latched ? 0.0F :
        static_cast<float>(clamp(cmd.steering, -1.0, 1.0));
      output.throttle = red_latched ? 0.0F :
        static_cast<float>(clamp(cmd.throttle, 0.0, speed_max));
      control_pub_->publish(output);
    }
    green_pub_->publish(std_msgs::msg::Bool().set__data(light == 1));
    red_pub_->publish(std_msgs::msg::Bool().set__data(light == 2));
    sign_pub_->publish(std_msgs::msg::String().set__data(decision.value_or("none")));
    state_pub_->publish(std_msgs::msg::String().set__data(mission_state));
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

    auto draw_curve = [&](const std::optional<cv::Vec3d> & curve) {
      if (!curve) return;
      std::vector<cv::Point> points;
      const int bottom = debug.roi.height - 1;
      for (int y = debug.hough.top_y; y <= bottom; y += 4) {
        const double x = evaluate_curve(*curve, y, debug.hough.top_y, bottom);
        if (std::isfinite(x) && x >= -debug.roi.width && x <= 2 * debug.roi.width) {
          points.emplace_back(cvRound(x) + debug.roi.x, y + debug.roi.y);
        }
      }
      if (points.size() >= 2) cv::polylines(frame, points, false, lane_color, 5, cv::LINE_AA);
    };
    draw_curve(debug.hough.left_curve);
    draw_curve(debug.hough.right_curve);

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

  static void draw_virtual_fork_target(
    cv::Mat & frame, const std::string & state, const std::string & decision,
    const Config & config)
  {
    if (decision.empty() ||
      (state != "OUT_FORK_SIGN_ADVANCE" && state != "OUT_FORK_COMMIT"))
    {
      return;
    }
    const cv::Point vehicle(frame.cols / 2, frame.rows - 1);
    const double side = decision == "LEFT" ? -1.0 : 1.0;
    const cv::Point target(
      std::clamp(cvRound(vehicle.x + side * config.fork_forced_error * vehicle.x),
        0, frame.cols - 1),
      std::clamp(cvRound(frame.rows * 0.42), 0, frame.rows - 1));
    const cv::Scalar color(255, 0, 255);
    cv::arrowedLine(frame, vehicle, target, color, 4, cv::LINE_AA, 0, 0.12);
    cv::circle(frame, target, 8, color, -1, cv::LINE_AA);
    put_outlined_text(frame, "VIRTUAL " + decision,
      cv::Point(std::max(4, target.x - 62), std::max(18, target.y - 12)), 0.5, color);
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
    for (const auto & segment : debug.hough.selected_segments) {
      cv::line(view, cv::Point(segment[0], segment[1]), cv::Point(segment[2], segment[3]),
        cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
    }
    auto draw_curve = [&](const std::optional<cv::Vec3d> & curve) {
      if (!curve) return;
      std::vector<cv::Point> points;
      const int bottom = view.rows - 1;
      for (int y = debug.hough.top_y; y <= bottom; y += 4) {
        const double x = evaluate_curve(*curve, y, debug.hough.top_y, bottom);
        if (std::isfinite(x) && x >= -view.cols && x <= 2 * view.cols) {
          points.emplace_back(cvRound(x), y);
        }
      }
      if (points.size() >= 2) {
        cv::polylines(view, points, false, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      }
    };
    draw_curve(debug.hough.left_curve);
    draw_curve(debug.hough.right_curve);
    cv::line(view, cv::Point(0, debug.hough.top_y),
      cv::Point(view.cols - 1, debug.hough.top_y), cv::Scalar(0, 160, 0), 1);

    const int vehicle_x = std::clamp(frame_width / 2 - debug.roi.x, 0, view.cols - 1);
    cv::line(view, cv::Point(vehicle_x, 0), cv::Point(vehicle_x, view.rows - 1),
      cv::Scalar(255, 255, 0), 1);
    const cv::Point steer_origin(vehicle_x, view.rows - 1);
    cv::arrowedLine(view, steer_origin,
      steering_tip(steer_origin, cmd.steering, static_cast<int>(view.rows * 0.45)),
      cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.2);
    put_outlined_text(view, "mask=STEERING " + std::to_string(view.cols) + "x" +
      std::to_string(view.rows),
      cv::Point(8, 20));
    std::ostringstream status;
    status.setf(std::ios::fixed); status.precision(2);
    status << "err=" << std::showpos << lane.center_error <<
      " curv=" << std::noshowpos << lane.curvature <<
      " steer=" << std::showpos << cmd.steering <<
      " pair=" << (lane.both_lanes ? "2" : "1") <<
      " branch=" << (lane.branch_pair_selected ? "Y" : "N") <<
      " multi=" << (lane.branch_scene_ambiguous ? "Y" : "N");
    put_outlined_text(view, status.str(), cv::Point(8, 42));
    return view;
  }

  void debug_loop() {
    try {
      debug_loop_impl();
    } catch (const cv::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "skip invalid debug frame after OpenCV error: %s", error.what());
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "skip invalid debug frame: %s", error.what());
    }
  }

  void debug_loop_impl() {
    cv::Mat frame;
    LaneDebug lane_debug;
    LaneObs lane;
    Command cmd;
    std::vector<Detection> detections;
    std::vector<int> markers;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::array<double, 4> light_roi;
    std::string state;
    std::string fork_decision;
    int light{0};
    Config config;
    rclcpp::Time frame_stamp{0, 0, RCL_ROS_TIME};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (latest_frame_.empty()) return;
      lane_debug = latest_lane_debug_; lane = latest_lane_;
      cmd = last_command_; state = state_name(state_);
      fork_decision = fork_decision_;
      markers = marker_ids_; marker_corners = marker_corners_;
      light = light_state_; config = config_;
      light_roi = light_roi_;
      frame = latest_frame_.clone();
      detections = detections_;
      frame_stamp = now();
    }
    frame = corrected_debug_frame(frame, config);
    draw_lane_debug(frame, lane_debug, lane, cmd);
    draw_virtual_fork_target(frame, state, fork_decision, config);
    const cv::Point roi_tl(
      cvRound(light_roi[0] * frame.cols), cvRound(light_roi[1] * frame.rows));
    const cv::Point roi_br(
      cvRound(light_roi[2] * frame.cols), cvRound(light_roi[3] * frame.rows));
    cv::rectangle(frame, roi_tl, roi_br, cv::Scalar(0, 255, 255), 2);
    put_outlined_text(frame, "NCNN light ROI",
      cv::Point(roi_tl.x + 4, std::max(16, roi_tl.y + 18)), 0.5,
      cv::Scalar(0, 255, 255));
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
      // OpenCV's polylines overload used here requires integer points. Passing
      // detectMarkers' Point2f vector raises an exception as soon as a marker
      // appears and used to terminate the whole C++ node. camera-freeze drew a
      // simple integer bounding box; keep that safe behavior in this path.
      std::vector<cv::Point> safe_corners;
      safe_corners.reserve(marker_corners[i].size());
      for (const auto & point : marker_corners[i]) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
        safe_corners.emplace_back(
          std::clamp(cvRound(point.x), 0, std::max(0, frame.cols - 1)),
          std::clamp(cvRound(point.y), 0, std::max(0, frame.rows - 1)));
      }
      if (safe_corners.empty() || i >= markers.size()) continue;
      const cv::Rect marker_box = cv::boundingRect(safe_corners);
      if (marker_box.width <= 0 || marker_box.height <= 0) continue;
      cv::rectangle(frame, marker_box, color, target ? 3 : 2, cv::LINE_AA);
      put_outlined_text(
        frame, "ID " + std::to_string(markers[i]),
        cv::Point(marker_box.x, std::max(14, marker_box.y - 6)), 0.55, color);
    }
    put_outlined_text(frame, "state=" + state, cv::Point(8, 22));
    std::ostringstream command_text;
    command_text.setf(std::ios::fixed); command_text.precision(2);
    command_text << "thr=" << cmd.throttle << " err=" << std::showpos << lane.center_error <<
      " curv=" << std::noshowpos << lane.curvature <<
      " scurv=" << std::showpos << lane.signed_curvature <<
      " steer=" << cmd.steering <<
      " pair=" << (lane.both_lanes ? "2" : "1") <<
      " branch=" << (lane.branch_pair_selected ? "Y" : "N") <<
      " multi=" << (lane.branch_scene_ambiguous ? "Y" : "N");
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
  std::mutex mutex_, image_mutex_, lane_mutex_, control_output_mutex_;
  sensor_msgs::msg::CompressedImage::SharedPtr pending_image_;
  LaneObs latest_lane_;
  LaneDebug latest_lane_debug_;
  cv::Mat latest_frame_;
  std::vector<Detection> detections_;
  std::array<double, 4> light_roi_{0.00, 0.00, 0.80, 0.85};
  std::vector<int> marker_ids_;
  std::vector<std::vector<cv::Point2f>> marker_corners_;
  std::deque<int> sign_history_;
  Command last_command_;
  bool target_marker_{false}, publish_debug_{false};
  bool has_started_{false}, aruco_stop_active_{false}, red_stop_armed_{false};
  std::atomic<bool> red_stop_latched_{false};
  double aruco_pause_started_{0.0};
  int light_state_{0}, green_streak_{0}, red_streak_{0}, aruco_arm_streak_{0};
  std::atomic<bool> lane_reset_requested_{false};
  bool fork_scene_observed_{false};
  int fork_pair_streak_{0}, fork_clear_streak_{0};
  uint64_t detection_sequence_{0}, last_light_sequence_{0};
  int32_t processed_image_sec_{0};
  uint32_t processed_image_nanosec_{0};
  rclcpp::Time light_received_{0, 0, RCL_ROS_TIME};
  MissionState state_{MissionState::OUT_WAIT_GREEN};
  RouteMode route_mode_{RouteMode::OUT};
  std::string fork_decision_, image_topic_, control_topic_, detections_topic_;
  std::string mission_state_topic_;
  double entered_{0.0}, debug_hz_{5.0}, perception_hz_{20.0}, control_hz_{20.0};
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sign_pub_, aruco_pub_, state_pub_;
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
