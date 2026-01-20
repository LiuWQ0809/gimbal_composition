#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_msgs/msg/string.hpp>
#include <recomo_controller/msg/tracked_object2_d.hpp>
#include <jc2804_gimbal_driver/msg/gimbal_command.hpp>
#include <jc2804_gimbal_driver/msg/gimbal_state.hpp>

#include <nlohmann/json.hpp>
#include <algorithm> // for clamp
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using json = nlohmann::json;

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
    
    // Safety & Smoothness Params
    this->declare_parameter("max_velocity_rpm", 5.0); // Limit max speed (lower than driver max 10)
    this->declare_parameter("max_step_rad", 0.05);    // Max position jump per tick (~2.8 deg)
    this->declare_parameter("limit_pitch_rad", 0.78); // Limit pitch to +/- 45 degrees
    this->declare_parameter("limit_yaw_rad", 1.57);   // Limit yaw to +/- 90 degrees
    
    // Subscribers
    tracking_sub_ = this->create_subscription<recomo_controller::msg::TrackedObject2D>(
      "/recomo/subject_tracking", 10,
      std::bind(&GimbalTrackingNode::trackingCallback, this, std::placeholders::_1));

    telemetry_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/recomo/rgb/telemetry", 10,
      std::bind(&GimbalTrackingNode::telemetryCallback, this, std::placeholders::_1));

    gimbal_state_sub_ = this->create_subscription<jc2804_gimbal_driver::msg::GimbalState>(
      "/gimbal/state", 10,
      std::bind(&GimbalTrackingNode::gimbalStateCallback, this, std::placeholders::_1));

    // Publisher
    gimbal_cmd_pub_ = this->create_publisher<jc2804_gimbal_driver::msg::GimbalCommand>(
      "/gimbal/command", 10);

    // Timer for control loop
    double rate = this->get_parameter("cmd_rate_hz").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&GimbalTrackingNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Gimbal Tracking Node Started.");
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

  void trackingCallback(const recomo_controller::msg::TrackedObject2D::SharedPtr msg)
  {
    last_tracking_msg_ = msg;
    last_tracking_time_ = this->now();
  }

  void gimbalStateCallback(const jc2804_gimbal_driver::msg::GimbalState::SharedPtr msg)
  {
    last_gimbal_state_ = msg;
    current_yaw_ = msg->yaw_position_rad;   // Assuming field name based on pattern
    current_pitch_ = msg->pitch_position_rad;
    // Reading field names from msg file:
    // float64 roll_position_rad
    // float64 pitch_position_rad
    // float64 yaw_position_rad (implied, will verify compile)
  }

  void controlLoop()
  {
    if (!last_gimbal_state_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for gimbal state...");
      return;
    }

    if (!last_tracking_msg_) {
      return; 
    }

    // Check freshness of tracking
    if ((this->now() - last_tracking_time_).seconds() > 0.5) {
      // Tracking lost or stale
      return;
    }

    if (last_tracking_msg_->confidence < this->get_parameter("confidence_threshold").as_double()) {
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
    double target_yaw = last_commanded_yaw_ + delta_yaw;
    double target_pitch = last_commanded_pitch_ + delta_pitch;
    
    // Safety Limits (Absolute Clamp)
    double limit_yaw = this->get_parameter("limit_yaw_rad").as_double();
    double limit_pitch = this->get_parameter("limit_pitch_rad").as_double();
    
    target_yaw = std::clamp(target_yaw, -limit_yaw, limit_yaw);
    target_pitch = std::clamp(target_pitch, -limit_pitch, limit_pitch);

    // Update Integrator
    last_commanded_yaw_ = target_yaw;
    last_commanded_pitch_ = target_pitch;

    // Publish Command
    jc2804_gimbal_driver::msg::GimbalCommand cmd;
    cmd.roll_rad = 0.0;
    cmd.pitch_rad = target_pitch;
    cmd.yaw_rad = target_yaw;
    
    double max_v = this->get_parameter("max_velocity_rpm").as_double();
    
    cmd.roll_velocity_rpm = max_v;
    cmd.pitch_velocity_rpm = max_v;
    cmd.yaw_velocity_rpm = max_v;

    gimbal_cmd_pub_->publish(cmd);

    RCLCPP_INFO(this->get_logger(), 
      "BBOX: [%u, %u, %u, %u] | Err(%d, %d)\n"
      "    State(Y:%.3f, P:%.3f) | Delta(Y:%.5f, P:%.5f) -> Target(Cmd)(Y:%.3f, P:%.3f)", 
      bbox.x_offset, bbox.y_offset, bbox.width, bbox.height,
      error_x, error_y, 
      current_yaw_, current_pitch_,
      delta_yaw, delta_pitch, 
      target_yaw, target_pitch);
  }

  // Member variables
  bool last_commanded_valid_ = false;
  double last_commanded_yaw_ = 0.0;
  double last_commanded_pitch_ = 0.0;

  rclcpp::Subscription<recomo_controller::msg::TrackedObject2D>::SharedPtr tracking_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
  rclcpp::Subscription<jc2804_gimbal_driver::msg::GimbalState>::SharedPtr gimbal_state_sub_;
  rclcpp::Publisher<jc2804_gimbal_driver::msg::GimbalCommand>::SharedPtr gimbal_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  recomo_controller::msg::TrackedObject2D::SharedPtr last_tracking_msg_;
  rclcpp::Time last_tracking_time_;
  
  jc2804_gimbal_driver::msg::GimbalState::SharedPtr last_gimbal_state_;
  double current_yaw_ = 0.0;
  double current_pitch_ = 0.0;
  
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
