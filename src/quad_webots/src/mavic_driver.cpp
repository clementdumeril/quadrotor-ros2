#include "quad_webots/mavic_driver.hpp"

#include <cmath>
#include <vector>

#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

namespace quad_webots
{

namespace
{
constexpr double kGravity = 9.81;
const char * kMotorNames[4] = {
  "front left propeller", "front right propeller", "rear left propeller", "rear right propeller"};

Eigen::Vector3d toEigen(const std::vector<double> & v) { return {v.at(0), v.at(1), v.at(2)}; }
}  // namespace

void MavicDriver::init(webots_ros2_driver::WebotsNode * node,
                       std::unordered_map<std::string, std::string> & /*parameters*/)
{
  node_ = node;
  dt_ = wb_robot_get_basic_time_step() / 1000.0;
  const int step_ms = static_cast<int>(wb_robot_get_basic_time_step());

  gps_ = wb_robot_get_device("gps");
  imu_ = wb_robot_get_device("inertial unit");
  gyro_ = wb_robot_get_device("gyro");
  wb_gps_enable(gps_, step_ms);
  wb_inertial_unit_enable(imu_, step_ms);
  wb_gyro_enable(gyro_, step_ms);
  for (int i = 0; i < 4; ++i) {
    motors_[i] = wb_robot_get_device(kMotorNames[i]);
    wb_motor_set_position(motors_[i], INFINITY);
    wb_motor_set_velocity(motors_[i], 0.0);
  }

  // Physical constants come from the Mavic2Pro PROTO (Webots R2025a); see config/mavic.yaml.
  mass_ = node_->declare_parameter("mass", 0.5);
  max_motor_speed_ = node_->declare_parameter("max_motor_speed", 576.0);
  cmd_timeout_ = node_->declare_parameter("cmd_timeout", 0.3);
  const double k_t = node_->declare_parameter("k_thrust", 2.6e-4);
  const double k_q = node_->declare_parameter("k_torque", 5.2e-6);
  const auto rx = node_->declare_parameter("rotor_x", std::vector<double>{0.143, 0.143, -0.089, -0.089});
  const auto ry = node_->declare_parameter("rotor_y", std::vector<double>{0.151, -0.151, 0.127, -0.127});
  const auto spin = node_->declare_parameter("rotor_spin", std::vector<int64_t>{1, -1, -1, 1});
  kp_rate_ = toEigen(node_->declare_parameter("kp_rate", std::vector<double>{0.05, 0.05, 0.02}));
  ki_rate_ = toEigen(node_->declare_parameter("ki_rate", std::vector<double>{0.1, 0.1, 0.02}));
  i_limit_ = toEigen(node_->declare_parameter("rate_integral_limit", std::vector<double>{0.05, 0.05, 0.02}));

  std::array<Rotor, 4> rotors{};
  for (int i = 0; i < 4; ++i) {
    rotors[i] = Rotor{rx.at(i), ry.at(i), static_cast<int>(spin.at(i))};
  }
  mixer_ = std::make_unique<Mixer>(rotors, k_t, k_q, max_motor_speed_);

  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/quad/odom", rclcpp::SensorDataQoS());
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/quad/imu", rclcpp::SensorDataQoS());
  motor_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/quad/motor_speeds", 10);
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
  cmd_sub_ = node_->create_subscription<quad_interfaces::msg::RatesThrust>(
    "/quad/cmd/rates_thrust", rclcpp::SensorDataQoS(),
    [this](const quad_interfaces::msg::RatesThrust::SharedPtr msg) {
      cmd_ = *msg;
      last_cmd_time_ = node_->get_clock()->now();
      armed_ = true;
    });

  RCLCPP_INFO(node_->get_logger(), "Mavic driver ready: step %.1f ms, hover thrust %.2f N, max %.2f N",
              dt_ * 1e3, mass_ * kGravity, 4.0 * mixer_->maxThrustPerRotor());
}

void MavicDriver::readState()
{
  const double * p = wb_gps_get_values(gps_);
  const double * v = wb_gps_get_speed_vector(gps_);
  const double * q = wb_inertial_unit_get_quaternion(imu_);  // x, y, z, w
  const double * w = wb_gyro_get_values(gyro_);
  p_ = Eigen::Vector3d(p[0], p[1], p[2]);
  v_world_ = Eigen::Vector3d(v[0], v[1], v[2]);
  q_ = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized();
  omega_ = Eigen::Vector3d(w[0], w[1], w[2]);
}

void MavicDriver::publishState(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = p_.x();
  odom.pose.pose.position.y = p_.y();
  odom.pose.pose.position.z = p_.z();
  odom.pose.pose.orientation.x = q_.x();
  odom.pose.pose.orientation.y = q_.y();
  odom.pose.pose.orientation.z = q_.z();
  odom.pose.pose.orientation.w = q_.w();
  // REP 105: the twist is expressed in the child (body) frame.
  const Eigen::Vector3d v_body = q_.conjugate() * v_world_;
  odom.twist.twist.linear.x = v_body.x();
  odom.twist.twist.linear.y = v_body.y();
  odom.twist.twist.linear.z = v_body.z();
  odom.twist.twist.angular.x = omega_.x();
  odom.twist.twist.angular.y = omega_.y();
  odom.twist.twist.angular.z = omega_.z();
  odom_pub_->publish(odom);

  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = "base_link";
  imu.orientation = odom.pose.pose.orientation;
  imu.angular_velocity = odom.twist.twist.angular;
  imu.linear_acceleration_covariance[0] = -1.0;  // no accelerometer on this model
  imu_pub_->publish(imu);

  geometry_msgs::msg::TransformStamped tf;
  tf.header = odom.header;
  tf.child_frame_id = "base_link";
  tf.transform.translation.x = p_.x();
  tf.transform.translation.y = p_.y();
  tf.transform.translation.z = p_.z();
  tf.transform.rotation = odom.pose.pose.orientation;
  tf_->sendTransform(tf);
}

Eigen::Vector3d MavicDriver::rateLoop(const Eigen::Vector3d & rates_des, double dt)
{
  const Eigen::Vector3d err = rates_des - omega_;
  rate_integral_ = (rate_integral_ + ki_rate_.cwiseProduct(err) * dt).cwiseMax(-i_limit_).cwiseMin(i_limit_);
  return kp_rate_.cwiseProduct(err) + rate_integral_;
}

void MavicDriver::step()
{
  readState();
  const rclcpp::Time now = node_->get_clock()->now();
  publishState(now);

  std::array<double, 4> speeds{0.0, 0.0, 0.0, 0.0};
  if (armed_) {
    double thrust = cmd_.thrust;
    Eigen::Vector3d rates_des(cmd_.body_rates.x, cmd_.body_rates.y, cmd_.body_rates.z);
    if ((now - last_cmd_time_).seconds() > cmd_timeout_) {
      // Failsafe: level the body and sink at a small fraction below hover thrust.
      const Eigen::Vector3d z_body = q_ * Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d tilt = q_.conjugate() * z_body.cross(Eigen::Vector3d::UnitZ());
      rates_des = Eigen::Vector3d(4.0 * tilt.x(), 4.0 * tilt.y(), 0.0);
      thrust = 0.9 * mass_ * kGravity;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "No command for %.2f s: failsafe descent", (now - last_cmd_time_).seconds());
      if (p_.z() < 0.15 && v_world_.norm() < 0.3) {
        armed_ = false;  // on the ground: stop the motors until the next command
        RCLCPP_INFO(node_->get_logger(), "Landed, motors off");
      }
    }
    if (armed_) {
      speeds = mixer_->speeds(thrust, rateLoop(rates_des, dt_));
    }
  } else {
    rate_integral_.setZero();
  }

  for (int i = 0; i < 4; ++i) {
    wb_motor_set_velocity(motors_[i], speeds[i]);
  }
  std_msgs::msg::Float64MultiArray m;
  m.data.assign(speeds.begin(), speeds.end());
  motor_pub_->publish(m);
}

}  // namespace quad_webots

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(quad_webots::MavicDriver, webots_ros2_driver::PluginInterface)
