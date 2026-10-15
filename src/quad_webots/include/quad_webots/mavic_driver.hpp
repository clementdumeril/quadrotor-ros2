#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <webots/types.h>
#include <webots/supervisor.h>
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
// For tests it also applies an external force (wind, unmodelled payload) published on
// /quad/disturbance/force, and records the 3D view to a video file on request (/quad/record).
class MavicDriver : public webots_ros2_driver::PluginInterface
{
public:
  void init(webots_ros2_driver::WebotsNode * node,
            std::unordered_map<std::string, std::string> & parameters) override;
  void step() override;

private:
  void readState();
  void logCenterOfMass();
  void applyRotorForces(const std::array<double, 4> & commanded);
  void publishState(const rclcpp::Time & stamp);
  Eigen::Vector3d rateLoop(const Eigen::Vector3d & rates_des, double dt);
  void startRecording(const std::string & file);
  void updateTrails();

  webots_ros2_driver::WebotsNode * node_{nullptr};
  double dt_{0.004};

  // Webots devices
  WbDeviceTag gps_{0}, imu_{0}, gyro_{0};
  std::array<WbDeviceTag, 4> motors_{};

  // Model and gains (from config/mavic.yaml)
  double hover_thrust_{4.9};
  double mass_{0.5}, max_motor_speed_{576.0}, cmd_timeout_{0.3};
  Eigen::Vector3d kp_rate_, ki_rate_, i_limit_;
  std::unique_ptr<Mixer> mixer_;

  // Explicit rotor model (forces applied through the supervisor)
  WbNodeRef self_{nullptr};
  double k_t_{2.6e-4}, k_q_{5.2e-6}, motor_tau_{0.03};
  std::array<std::array<double, 3>, 4> thrust_points_{};
  std::array<int, 4> spin_{};
  std::array<double, 4> rotor_speed_{};

  // State (world frame ENU, body frame FLU)
  Eigen::Vector3d p_, v_world_, omega_;
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d rate_integral_{Eigen::Vector3d::Zero()};
  double ground_z_{std::nan("")};

  // Latest command
  quad_interfaces::msg::RatesThrust cmd_;
  rclcpp::Time last_cmd_time_;
  bool armed_{false};
  bool com_logged_{false};

  rclcpp::Subscription<quad_interfaces::msg::RatesThrust>::SharedPtr cmd_sub_;
  // Identification only: raw propeller speeds [rad/s], bypassing the rate loop and the mixer.
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr raw_sub_;
  std::array<double, 4> raw_speeds_{};
  rclcpp::Time last_raw_time_;
  bool raw_received_{false};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pub_;

  // Test inputs: external force in the world frame [N], and video recording of the 3D view.
  Eigen::Vector3d disturbance_{Eigen::Vector3d::Zero()};
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr disturbance_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr record_sub_;
  bool recording_{false};
  // While recording: fixed overview camera, and two polylines drawn in the 3D view, the
  // reference (/quad/setpoint, grey) and the flown path (blue).
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_sub_;
  Eigen::Vector3d setpoint_{Eigen::Vector3d::Constant(std::nan(""))};
  static constexpr int kTrailLayers = 5;
  std::array<WbNodeRef, 2 * kTrailLayers> trail_nodes_{};
  std::array<WbFieldRef, 2 * kTrailLayers> trail_points_{}, trail_index_{};
  int trail_count_{0}, trail_tick_{0};
  std::vector<double> view_position_, view_target_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

}  // namespace quad_webots
