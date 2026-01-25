#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_msgs/msg/string.hpp>
#include <recomo_msgs/msg/tracking.hpp>
#include <jc2804_gimbal_driver/msg/gimbal_command.hpp>
#include <jc2804_gimbal_driver/msg/gimbal_state.hpp>
#include <ronin_rs4_driver/msg/ronin_rs4_control.hpp>
#include <ronin_rs4_driver/msg/ronin_rs4_status.hpp>

#include <nlohmann/json.hpp>
#include <array>
#include <algorithm> // for clamp
#include <chrono>
#include <cctype>
#include <cmath>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using json = nlohmann::json;

namespace {

constexpr double kPi = 3.14159265358979323846;

double DegFromRad(double rad) {
  return rad * 180.0 / kPi;
}

double RadFromDeg(double deg) {
  return deg * kPi / 180.0;
}

}  // namespace

enum class GimbalDriver {
  kJc2804,
  kRs4,
};

enum class Rs4ControlMode {
  kAttitude,
  kJoint,
};

class GimbalTrackingNode : public rclcpp::Node
{
public:
  GimbalTrackingNode() : Node("gimbal_tracking_node")
  {
    // Parameters
    this->declare_parameter("image_width", 1920);
    this->declare_parameter("image_height", 1080);
    this->declare_parameter("kp_yaw", 0.0005);
    this->declare_parameter("kp_pitch", 0.0005);
    this->declare_parameter("ki_yaw", 0.0);
    this->declare_parameter("ki_pitch", 0.0);
    this->declare_parameter("kd_yaw", 0.0);
    this->declare_parameter("kd_pitch", 0.0);
    this->declare_parameter("deadband_x", 10);
    this->declare_parameter("deadband_y", 10);
    this->declare_parameter("confidence_threshold", 0.3);
    this->declare_parameter("cmd_rate_hz", 20.0);
    this->declare_parameter("default_focal_length", 1000.0); // Fallback pixel focal length
    std::string driver = this->declare_parameter("gimbal_driver", "jc2804");
    std::string rs4_control_mode = this->declare_parameter("rs4_control_mode", "attitude");
    topic_gimbal_state_ =
      this->declare_parameter<std::string>("topics.gimbal_state", "/gimbal/state");
    topic_gimbal_command_ =
      this->declare_parameter<std::string>("topics.gimbal_command", "/gimbal/command");
    topic_rs4_state_ = this->declare_parameter<std::string>(
      "topics.rs4_status", "/ronin_rs4_driver/status/state");
    topic_rs4_command_ = this->declare_parameter<std::string>(
      "topics.rs4_command", "/ronin_rs4_driver/cmd/control");
    rs4_axis_map_ = ParseAxisMap(
      this->declare_parameter<std::vector<int64_t>>(
        "rs4_axis_map_from_gimbal", std::vector<int64_t>{2, 0, 1}),
      {2, 0, 1});
    rs4_axis_map_inv_ = InvertAxisMap(rs4_axis_map_);
    rs4_axis_sign_ = ParseAxisVector(
      this->declare_parameter<std::vector<double>>(
        "rs4_axis_sign", std::vector<double>{1.0, 1.0, 1.0}),
      {1.0, 1.0, 1.0});
    rs4_axis_offset_rad_ = ParseAxisVector(
      this->declare_parameter<std::vector<double>>(
        "rs4_axis_zero_offset_rad", std::vector<double>{0.0, 0.0, 0.0}),
      {0.0, 0.0, 0.0});
    driver_ = ParseDriver(driver);
    rs4_control_mode_ = ParseRs4ControlMode(rs4_control_mode);
    
    // Safety & Smoothness Params
    this->declare_parameter("max_velocity_rpm", 5.0); // Limit max speed (lower than driver max 10)
    this->declare_parameter("max_step_rad", 0.05);    // Max position jump per tick (~2.8 deg)
    this->declare_parameter("limit_pitch_rad", 0.78); // Limit pitch to +/- 45 degrees
    this->declare_parameter("limit_yaw_rad", 1.57);   // Limit yaw to +/- 90 degrees
    
    // Subscribers
    tracking_sub_ = this->create_subscription<recomo_msgs::msg::Tracking>(
      "/recomo/subject_tracking", 10,
      std::bind(&GimbalTrackingNode::trackingCallback, this, std::placeholders::_1));

    telemetry_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/recomo/rgb/telemetry", 10,
      std::bind(&GimbalTrackingNode::telemetryCallback, this, std::placeholders::_1));

    if (driver_ == GimbalDriver::kJc2804) {
      gimbal_state_sub_ = this->create_subscription<jc2804_gimbal_driver::msg::GimbalState>(
        topic_gimbal_state_, 10,
        std::bind(&GimbalTrackingNode::gimbalStateCallback, this, std::placeholders::_1));
      gimbal_cmd_pub_ = this->create_publisher<jc2804_gimbal_driver::msg::GimbalCommand>(
        topic_gimbal_command_, 10);
    } else {
      rs4_state_sub_ = this->create_subscription<ronin_rs4_driver::msg::RoninRs4Status>(
        topic_rs4_state_, 10,
        std::bind(&GimbalTrackingNode::rs4StateCallback, this, std::placeholders::_1));
      rs4_cmd_pub_ = this->create_publisher<ronin_rs4_driver::msg::RoninRs4Control>(
        topic_rs4_command_, 10);
    }

    // Timer for control loop
    double rate = this->get_parameter("cmd_rate_hz").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&GimbalTrackingNode::controlLoop, this));

    const char *driver_name = driver_ == GimbalDriver::kRs4 ? "ronin_rs4" : "jc2804";
    const char *rs4_mode = rs4_control_mode_ == Rs4ControlMode::kAttitude ? "attitude" : "joint";
    RCLCPP_INFO(this->get_logger(),
      "Gimbal Tracking Node Started (driver=%s, rs4_mode=%s).", driver_name, rs4_mode);
  }

private:
  void telemetryCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    try {
      auto data = json::parse(msg->data);
      
      // Update image dimensions if available
      if (data.contains("width") && data.contains("height")) {
        int w = data["width"].get<int>();
        int h = data["height"].get<int>();
        // Only update if changed and valid
        if (w > 0 && h > 0 && (w != current_width_ || h != current_height_)) {
          current_width_ = w;
          current_height_ = h;
          RCLCPP_INFO(this->get_logger(), "Updated image size from telemetry: %dx%d", w, h);
        }
      }

      // Update focal length if available
      if (data.contains("fx")) {
        double fx = data["fx"].get<double>();
        if (fx > 1.0) { // Basic sanity check
          current_fx_ = fx;
          // Optionally update fy too if needed, but fx is sufficient for H-FOV dominated yaw
        }
      }
      
      // Check for 'cx' and 'cy' if we want to update principal point
      // For now, center of image W/H is usually close enough approximation for tracking 
      // unless cropped significantly off-center.
      
    } catch (const json::exception& e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        "JSON parse error in telemetry: %s", e.what());
    }
  }

  void trackingCallback(const recomo_msgs::msg::Tracking::SharedPtr msg)
  {
    last_tracking_msg_ = msg;
    last_tracking_time_ = this->now();
  }

  void gimbalStateCallback(const jc2804_gimbal_driver::msg::GimbalState::SharedPtr msg)
  {
    current_roll_ = msg->roll_position_rad;
    current_pitch_ = msg->pitch_position_rad;
    current_yaw_ = msg->yaw_position_rad;
    have_gimbal_state_ = true;
  }

  void rs4StateCallback(const ronin_rs4_driver::msg::RoninRs4Status::SharedPtr msg)
  {
    geometry_msgs::msg::Vector3 source = msg->joint_deg;
    if (rs4_control_mode_ == Rs4ControlMode::kAttitude) {
      if ((msg->valid_mask & ronin_rs4_driver::msg::RoninRs4Status::VALID_ATTITUDE) == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "Received RS4 state but VALID_ATTITUDE bit is missing. Mask: %u", msg->valid_mask);
        return;
      }
      source = msg->attitude_deg;
    } else {
      if ((msg->valid_mask & ronin_rs4_driver::msg::RoninRs4Status::VALID_JOINT) == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "Received RS4 state but VALID_JOINT bit is missing. Mask: %u", msg->valid_mask);
        return;
      }
    }

    if (!rs4_connected_once_) {
        RCLCPP_INFO(this->get_logger(), "RS4 State Received and Valid. Mask: %u", msg->valid_mask);
        rs4_connected_once_ = true;
    }

    std::array<double, 3> rs4_rad{
      RadFromDeg(source.x),
      RadFromDeg(source.y),
      RadFromDeg(source.z)
    };
    for (size_t i = 0; i < 3; ++i) {
      rs4_rad[i] = rs4_axis_sign_[i] * (rs4_rad[i] - rs4_axis_offset_rad_[i]);
    }

    std::array<double, 3> gimbal_rad{};
    for (size_t i = 0; i < 3; ++i) {
      gimbal_rad[i] = rs4_rad[static_cast<size_t>(rs4_axis_map_inv_[i])];
    }

    current_roll_ = gimbal_rad[0];
    current_pitch_ = gimbal_rad[1];
    current_yaw_ = gimbal_rad[2];
    have_gimbal_state_ = true;
  }

  void controlLoop()
  {
    if (!have_gimbal_state_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for gimbal state...");
      return;
    }

    if (!last_tracking_msg_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for tracking message...");
      return; 
    }

    // Check freshness of tracking
    if ((this->now() - last_tracking_time_).seconds() > 0.5) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Tracking message stale (>0.5s)");
      // Tracking lost or stale
      return;
    }

    if (last_tracking_msg_->confidence < this->get_parameter("confidence_threshold").as_double()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        "Tracking confidence low: %.2f < %.2f", 
        last_tracking_msg_->confidence, 
        this->get_parameter("confidence_threshold").as_double());
      return;
    }

    if (last_tracking_msg_->state == "lost") {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        "Tracking state is 'lost'. Gimbal holding position.");
      return;
    }

    // Determine current geometry usage
    int img_w = (current_width_ > 0) ? current_width_ : this->get_parameter("image_width").as_int();
    int img_h = (current_height_ > 0) ? current_height_ : this->get_parameter("image_height").as_int();
    
    // Determine focal length for Kp scaling
    // Default or configured Kp is usually tuned for specific resolution/FOV.
    // Theoretical Kp = 1 / fx. 
    // We can interpret the existing 'kp_yaw' parameter as a 'Loop Gain' factor (around 0.5-0.8)
    // applied to the theoretical Kp.
    // Or we can just use the provided Kp if no fx is available.
    
    double active_fx = current_fx_;
    if (active_fx <= 0.0) {
        active_fx = this->get_parameter("default_focal_length").as_double();
    }
    
    // Fallback if still invalid
    if (active_fx <= 1.0) active_fx = 1000.0; 

    // Calculate dynamic Kp based on focal length
    // User configured Kp in params.yaml (0.0005) is roughly 0.5 / 1000.
    // So let's use the param 'kp_yaw' as the 'Reference Kp' for the 'default_focal_length' 
    // and scale it if focal length changes (e.g. zooming).
    // Or simpler: User wants use intrinsics. Let's make Kp = param_gain / fx.
    // Since we don't want to break existing logic abruptly, let's keep using kp_yaw as provided
    // BUT scale it if we detect a zoom (fx change).
    // Let's assume the params.yaml Kp is tuned for the 'default_focal_length'.
    // Kp_new = Kp_ref * (f_ref / f_current)
    // NOTE: Angle = Error / fx. So gain should proportional to 1/fx.
    // So yes, if f increases (zoom in), pixels per degree increases, so we need smaller Kp (rad/pixel).
    
    double default_fx = this->get_parameter("default_focal_length").as_double();
    double scale_factor = default_fx / active_fx;
    
    // Apply scale factor to gains
    double kp_yaw = this->get_parameter("kp_yaw").as_double() * scale_factor;
    double ki_yaw = this->get_parameter("ki_yaw").as_double() * scale_factor;
    double kd_yaw = this->get_parameter("kd_yaw").as_double() * scale_factor;

    double kp_pitch = this->get_parameter("kp_pitch").as_double() * scale_factor;
    double ki_pitch = this->get_parameter("ki_pitch").as_double() * scale_factor;
    double kd_pitch = this->get_parameter("kd_pitch").as_double() * scale_factor;

    int center_x = img_w / 2;
    int center_y = img_h / 2;

    auto &bbox = last_tracking_msg_->bbox;
    int box_center_x = bbox.x_offset + bbox.width / 2;
    int box_center_y = bbox.y_offset + bbox.height / 2;

    int error_x = center_x - box_center_x; 
    int error_y = center_y - box_center_y; 

    // Deadband
    if (std::abs(error_x) < this->get_parameter("deadband_x").as_int()) error_x = 0;
    if (std::abs(error_y) < this->get_parameter("deadband_y").as_int()) error_y = 0;
    
    // Time delta
    rclcpp::Time now = this->now();
    double dt = (last_loop_time_.nanoseconds() == 0) ? 0.05 : (now - last_loop_time_).seconds();
    if (dt <= 0.001) dt = 0.05; // Safety minimum
    last_loop_time_ = now;

    // Initialize last_commanded if this is the first run
    if (!last_commanded_valid_) {
        last_commanded_yaw_ = current_yaw_;
        last_commanded_pitch_ = current_pitch_;
        last_commanded_valid_ = true;
        
        // Reset PID state
        prev_error_x_ = error_x;
        prev_error_y_ = error_y;
        integral_x_ = 0.0;
        integral_y_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Initialized Command Integrator at Yaw: %.3f, Pitch: %.3f", last_commanded_yaw_, last_commanded_pitch_);
    }

    // PID Calculations
    // ----------------
    
    // Yaw PID
    integral_x_ += error_x * dt;
    double derivative_x = (error_x - prev_error_x_) / dt;
    prev_error_x_ = error_x;
    
    double delta_yaw = (kp_yaw * error_x) + (ki_yaw * integral_x_) + (kd_yaw * derivative_x);

    // Pitch PID
    integral_y_ += error_y * dt;
    double derivative_y = (error_y - prev_error_y_) / dt;
    prev_error_y_ = error_y;
    
    double delta_pitch = (kp_pitch * error_y) + (ki_pitch * integral_y_) + (kd_pitch * derivative_y);

    // Safety Step Limit (Crucial for Speed Control)
    double max_step = this->get_parameter("max_step_rad").as_double();
    delta_yaw = std::clamp(delta_yaw, -max_step, max_step);
    delta_pitch = std::clamp(delta_pitch, -max_step, max_step);

    // Integrate Position
    double base_yaw = last_commanded_yaw_;
    double base_pitch = last_commanded_pitch_;
    if (driver_ == GimbalDriver::kRs4 && rs4_control_mode_ == Rs4ControlMode::kAttitude) {
      base_yaw = current_yaw_;
      base_pitch = current_pitch_;
    }
    double target_yaw = base_yaw + delta_yaw;
    double target_pitch = base_pitch + delta_pitch;
    
    // Safety Limits (Absolute Clamp)
    double limit_yaw = this->get_parameter("limit_yaw_rad").as_double();
    double limit_pitch = this->get_parameter("limit_pitch_rad").as_double();
    
    target_yaw = std::clamp(target_yaw, -limit_yaw, limit_yaw);
    target_pitch = std::clamp(target_pitch, -limit_pitch, limit_pitch);

    // Update Integrator
    last_commanded_yaw_ = target_yaw;
    last_commanded_pitch_ = target_pitch;

    // Publish Command
    if (driver_ == GimbalDriver::kJc2804) {
      jc2804_gimbal_driver::msg::GimbalCommand cmd;
      cmd.roll_rad = 0.0;
      cmd.pitch_rad = target_pitch;
      cmd.yaw_rad = target_yaw;
      
      double max_v = this->get_parameter("max_velocity_rpm").as_double();
      
      cmd.roll_velocity_rpm = max_v;
      cmd.pitch_velocity_rpm = max_v;
      cmd.yaw_velocity_rpm = max_v;

      gimbal_cmd_pub_->publish(cmd);
    } else {
      ronin_rs4_driver::msg::RoninRs4Control cmd;
      cmd.command = ronin_rs4_driver::msg::RoninRs4Control::CMD_POSITION;

      std::array<double, 3> gimbal_rad{0.0, target_pitch, target_yaw};
      std::array<double, 3> rs4_rad{};
      for (size_t i = 0; i < 3; ++i) {
        rs4_rad[i] = gimbal_rad[static_cast<size_t>(rs4_axis_map_[i])];
        rs4_rad[i] = rs4_axis_sign_[i] * rs4_rad[i] + rs4_axis_offset_rad_[i];
      }

      cmd.position_deg.x = DegFromRad(rs4_rad[0]);
      cmd.position_deg.y = DegFromRad(rs4_rad[1]);
      cmd.position_deg.z = DegFromRad(rs4_rad[2]);

      rs4_cmd_pub_->publish(cmd);
    }

    RCLCPP_INFO(this->get_logger(), 
      "BBOX: [%u, %u, %u, %u] | Err(%d, %d)\n"
      "    State(Y:%.3f, P:%.3f) | Delta(Y:%.5f, P:%.5f) -> Target(Cmd)(Y:%.3f, P:%.3f)", 
      bbox.x_offset, bbox.y_offset, bbox.width, bbox.height,
      error_x, error_y, 
      current_yaw_, current_pitch_,
      delta_yaw, delta_pitch, 
      target_yaw, target_pitch);
  }

  GimbalDriver ParseDriver(const std::string &driver) {
    std::string normalized = driver;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "rs4" || normalized == "ronin_rs4" || normalized == "dji_rs4") {
      return GimbalDriver::kRs4;
    }
    if (normalized != "jc2804") {
      RCLCPP_WARN(this->get_logger(),
        "Unknown gimbal_driver '%s', defaulting to jc2804.", driver.c_str());
    }
    return GimbalDriver::kJc2804;
  }

  Rs4ControlMode ParseRs4ControlMode(const std::string &mode) {
    std::string normalized = mode;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "attitude" || normalized == "world") {
      return Rs4ControlMode::kAttitude;
    }
    if (normalized == "joint") {
      return Rs4ControlMode::kJoint;
    }
    RCLCPP_WARN(this->get_logger(),
      "Unknown rs4_control_mode '%s', defaulting to attitude.", mode.c_str());
    return Rs4ControlMode::kAttitude;
  }

  std::array<int, 3> ParseAxisMap(
    const std::vector<int64_t> &map_param, const std::array<int, 3> &fallback) {
    if (map_param.size() == 3) {
      std::array<int, 3> map{};
      for (size_t i = 0; i < 3; ++i) {
        map[i] = static_cast<int>(map_param[i]);
      }
      if (IsValidAxisMap(map)) {
        return map;
      }
      RCLCPP_WARN(this->get_logger(),
        "Invalid rs4_axis_map_from_gimbal; using default.");
    } else {
      RCLCPP_WARN(this->get_logger(),
        "rs4_axis_map_from_gimbal should have 3 entries; using default.");
    }
    return fallback;
  }

  std::array<int, 3> InvertAxisMap(const std::array<int, 3> &map) const {
    std::array<int, 3> inv{};
    for (size_t i = 0; i < 3; ++i) {
      inv[static_cast<size_t>(map[i])] = static_cast<int>(i);
    }
    return inv;
  }

  bool IsValidAxisMap(const std::array<int, 3> &map) const {
    std::array<int, 3> tmp = map;
    std::sort(tmp.begin(), tmp.end());
    return tmp[0] == 0 && tmp[1] == 1 && tmp[2] == 2;
  }

  std::array<double, 3> ParseAxisVector(
    const std::vector<double> &vec, const std::array<double, 3> &fallback) {
    if (vec.size() != 3) {
      RCLCPP_WARN(this->get_logger(), "Axis vector size != 3; using defaults.");
      return fallback;
    }
    return {vec[0], vec[1], vec[2]};
  }

  // Member variables
  bool last_commanded_valid_ = false;
  double last_commanded_yaw_ = 0.0;
  double last_commanded_pitch_ = 0.0;
  GimbalDriver driver_{GimbalDriver::kJc2804};
  Rs4ControlMode rs4_control_mode_{Rs4ControlMode::kAttitude};
  std::string topic_gimbal_state_;
  std::string topic_gimbal_command_;
  std::string topic_rs4_state_;
  std::string topic_rs4_command_;
  std::array<int, 3> rs4_axis_map_{2, 0, 1};
  std::array<int, 3> rs4_axis_map_inv_{0, 1, 2};
  std::array<double, 3> rs4_axis_sign_{1.0, 1.0, 1.0};
  std::array<double, 3> rs4_axis_offset_rad_{0.0, 0.0, 0.0};

  rclcpp::Subscription<recomo_msgs::msg::Tracking>::SharedPtr tracking_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
  rclcpp::Subscription<jc2804_gimbal_driver::msg::GimbalState>::SharedPtr gimbal_state_sub_;
  rclcpp::Publisher<jc2804_gimbal_driver::msg::GimbalCommand>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Subscription<ronin_rs4_driver::msg::RoninRs4Status>::SharedPtr rs4_state_sub_;
  rclcpp::Publisher<ronin_rs4_driver::msg::RoninRs4Control>::SharedPtr rs4_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  recomo_msgs::msg::Tracking::SharedPtr last_tracking_msg_;
  rclcpp::Time last_tracking_time_;
  
  bool have_gimbal_state_ = false;
  bool rs4_connected_once_ = false;
  double current_roll_ = 0.0;
  double current_pitch_ = 0.0;
  double current_yaw_ = 0.0;
  
  // Dynamic parameters from telemetry
  int current_width_ = 0;
  int current_height_ = 0;
  double current_fx_ = 0.0;

  // PID State
  double prev_error_x_ = 0.0;
  double prev_error_y_ = 0.0;
  double integral_x_ = 0.0;
  double integral_y_ = 0.0;
  rclcpp::Time last_loop_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalTrackingNode>());
  rclcpp::shutdown();
  return 0;
}
