#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <webots/types.h>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

#include "quad_interfaces/msg/rates_thrust.hpp"
#include "quad_webots/mixer.hpp"

namespace quad_webots
{

// Webots side of the stack. Every simulation step it
//   1. reads GPS, inertial unit and gyro, publishes /quad/odom, /quad/imu and the TF;
//   2. tracks the latest /quad/cmd/rates_thrust with a body-rate PI loop;
//   3. turns thrust + torques into the four propeller speeds (mixer).
// Before the first command the motors stay off; if commands stop for longer
// than cmd_timeout the drone levels itself and descends slowly.
class MavicDriver : public webots_ros2_driver::PluginInterface
{
public:
  void init(webots_ros2_driver::WebotsNode * node,
            std::unordered_map<std::string, std::string> & parameters) override;
  void step() override;

private:
  void readState();
  void publishState(const rclcpp::Time & stamp);
  Eigen::Vector3d rateLoop(const Eigen::Vector3d & rates_des, double dt);

  webots_ros2_driver::WebotsNode * node_{nullptr};
  double dt_{0.004};

  // Webots devices
  WbDeviceTag gps_{0}, imu_{0}, gyro_{0};
  std::array<WbDeviceTag, 4> motors_{};

  // Model and gains (from config/mavic.yaml)
  double mass_{0.5}, max_motor_speed_{576.0}, cmd_timeout_{0.3};
  Eigen::Vector3d kp_rate_, ki_rate_, i_limit_;
  std::unique_ptr<Mixer> mixer_;

  // State (world frame ENU, body frame FLU)
  Eigen::Vector3d p_, v_world_, omega_;
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d rate_integral_{Eigen::Vector3d::Zero()};

  // Latest command
  quad_interfaces::msg::RatesThrust cmd_;
  rclcpp::Time last_cmd_time_;
  bool armed_{false};

  rclcpp::Subscription<quad_interfaces::msg::RatesThrust>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

}  // namespace quad_webots
