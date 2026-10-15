// Cascaded position controller: position -> velocity -> acceleration -> attitude -> body rates.
// It follows /quad/reference with feedforward of the reference velocity and acceleration;
// the saturations act on the feedback terms only, so fast trajectories are not clipped.
// The attitude part follows the geometric controller of Lee, Leok & McClamroch (CDC 2010):
// the desired thrust direction and yaw define R_des, and the attitude error
// e_R = 1/2 vee(R_des^T R - R^T R_des) is turned into body-rate commands,
//   w = -k_att e_R + R^T R_des w_des,
// where the feedforward rates w_des follow from the reference jerk (differential flatness,
// Mellinger & Kumar, ICRA 2011): the thrust axis turns at h = (m/|F|)(j - (z_b . j) z_b),
// so w_des = (-h . y_b, h . x_b, 0) for a constant yaw. Gains can be changed while flying
// (ros2 param set), for tuning.
// Output: /quad/cmd/rates_thrust (collective thrust + body rates), the same interface as
// the NMPC, so the two controllers can be swapped without touching the simulator side.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "quad_control/reference.hpp"
#include "quad_interfaces/msg/rates_thrust.hpp"
#include "quad_interfaces/msg/reference_trajectory.hpp"

namespace
{
using quad_control::kGravity;

Eigen::Vector3d vec3(const std::vector<double> & v) { return {v.at(0), v.at(1), v.at(2)}; }

Eigen::Vector3d vee(const Eigen::Matrix3d & S) { return {S(2, 1), S(0, 2), S(1, 0)}; }

Eigen::Vector3d clampNorm(const Eigen::Vector3d & v, double max_norm)
{
  const double n = v.norm();
  return n > max_norm ? v * (max_norm / n) : v;
}
}  // namespace

class PositionController : public rclcpp::Node
{
public:
  PositionController() : Node("position_controller")
  {
    mass_ = declare_parameter("mass", 0.5);
    rate_hz_ = declare_parameter("rate_hz", 100.0);
    kp_pos_ = vec3(declare_parameter("kp_pos", std::vector<double>{1.5, 1.5, 2.0}));
    kp_vel_ = vec3(declare_parameter("kp_vel", std::vector<double>{3.0, 3.0, 4.0}));
    ki_vel_ = vec3(declare_parameter("ki_vel", std::vector<double>{0.3, 0.3, 1.0}));
    kp_att_ = vec3(declare_parameter("kp_att", std::vector<double>{6.0, 6.0, 3.0}));
    max_vel_xy_ = declare_parameter("max_vel_xy", 2.0);
    max_vel_z_ = declare_parameter("max_vel_z", 1.0);
    max_tilt_ = declare_parameter("max_tilt_deg", 30.0) * M_PI / 180.0;
    max_rate_ = vec3(declare_parameter("max_rate", std::vector<double>{3.0, 3.0, 1.5}));
    max_thrust_ = declare_parameter("max_thrust", 2.0 * mass_ * kGravity);
    rate_ff_ = declare_parameter("rate_feedforward", true);
    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
      for (const auto & prm : params) {
        const std::string & n = prm.get_name();
        if (n == "kp_pos") kp_pos_ = vec3(prm.as_double_array());
        else if (n == "kp_vel") kp_vel_ = vec3(prm.as_double_array());
        else if (n == "ki_vel") ki_vel_ = vec3(prm.as_double_array());
        else if (n == "kp_att") kp_att_ = vec3(prm.as_double_array());
        else if (n == "rate_feedforward") rate_ff_ = prm.as_bool();
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/quad/odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr m) { odom_ = *m; have_odom_ = true; });
    ref_sub_ = create_subscription<quad_interfaces::msg::ReferenceTrajectory>(
      "/quad/reference", 10,
      [this](const quad_interfaces::msg::ReferenceTrajectory::SharedPtr m) { ref_.set(*m); });
    cmd_pub_ = create_publisher<quad_interfaces::msg::RatesThrust>("/quad/cmd/rates_thrust", rclcpp::SensorDataQoS());
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "/quad/controller/enable",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
        enabled_ = req->data;
        vel_integral_.setZero();
        res->success = true;
        res->message = enabled_ ? "controller enabled" : "controller disabled";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });
    // Timer on the node clock: with use_sim_time it follows the simulation, not the wall clock.
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz_),
                                  [this] { update(); });
  }

private:
  void update()
  {
    if (!enabled_ || !have_odom_ || !ref_.valid()) {
      return;
    }
    const double dt = 1.0 / rate_hz_;
    const auto & P = odom_.pose.pose;
    const Eigen::Vector3d p(P.position.x, P.position.y, P.position.z);
    const Eigen::Quaterniond q(P.orientation.w, P.orientation.x, P.orientation.y, P.orientation.z);
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const auto & T = odom_.twist.twist.linear;
    const Eigen::Vector3d v = R * Eigen::Vector3d(T.x, T.y, T.z);  // body -> world

    const double t = now().seconds();
    const quad_control::RefPoint ref = ref_.at(t);

    // Position -> velocity command: reference velocity + saturated position feedback.
    Eigen::Vector3d v_fb = kp_pos_.cwiseProduct(ref.p - p);
    const Eigen::Vector2d v_xy = clampNorm(Eigen::Vector3d(v_fb.x(), v_fb.y(), 0.0), max_vel_xy_).head<2>();
    v_fb = Eigen::Vector3d(v_xy.x(), v_xy.y(), std::clamp(v_fb.z(), -max_vel_z_, max_vel_z_));
    const Eigen::Vector3d v_cmd = ref.v + v_fb;

    // Velocity -> desired acceleration: reference acceleration + PI (the integral absorbs a
    // wrong mass, wind or a payload).
    const Eigen::Vector3d e_v = v_cmd - v;
    vel_integral_ = (vel_integral_ + e_v * dt).cwiseMax(-2.0).cwiseMin(2.0);
    Eigen::Vector3d a = ref.a + kp_vel_.cwiseProduct(e_v) + ki_vel_.cwiseProduct(vel_integral_);
    a.z() = std::max(a.z() + kGravity, 0.2 * kGravity);
    const double a_xy_max = a.z() * std::tan(max_tilt_);
    const Eigen::Vector3d a_xy = clampNorm(Eigen::Vector3d(a.x(), a.y(), 0.0), a_xy_max);
    a = Eigen::Vector3d(a_xy.x(), a_xy.y(), a.z());

    // Desired attitude from the thrust direction and the yaw reference.
    const Eigen::Vector3d F = mass_ * a;
    const Eigen::Matrix3d R_des = quad_control::attitudeFromAcceleration(F, ref.yaw);

    // Feedforward body rates from the reference jerk (central difference of the reference).
    Eigen::Vector3d w_des = Eigen::Vector3d::Zero();
    if (rate_ff_) {
      constexpr double h = 0.01;
      const Eigen::Vector3d jerk = (ref_.at(t + h).a - ref_.at(t - h).a) / (2.0 * h);
      const Eigen::Vector3d zb = R_des.col(2);
      const Eigen::Vector3d hw = mass_ / F.norm() * (jerk - zb.dot(jerk) * zb);
      w_des = Eigen::Vector3d(-hw.dot(R_des.col(1)), hw.dot(R_des.col(0)), 0.0);
    }

    const Eigen::Vector3d e_R = 0.5 * vee(R_des.transpose() * R - R.transpose() * R_des);
    const Eigen::Vector3d rates = (-kp_att_.cwiseProduct(e_R) + R.transpose() * R_des * w_des)
                                    .cwiseMax(-max_rate_).cwiseMin(max_rate_);
    const double thrust = std::clamp(F.dot(R.col(2)), 0.0, max_thrust_);

    quad_interfaces::msg::RatesThrust cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.thrust = thrust;
    cmd.body_rates.x = rates.x();
    cmd.body_rates.y = rates.y();
    cmd.body_rates.z = rates.z();
    cmd_pub_->publish(cmd);
  }

  double mass_, rate_hz_, max_vel_xy_, max_vel_z_, max_tilt_, max_thrust_;
  bool rate_ff_{true};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  Eigen::Vector3d kp_pos_, kp_vel_, ki_vel_, kp_att_, max_rate_;
  Eigen::Vector3d vel_integral_{Eigen::Vector3d::Zero()};
  bool enabled_{true}, have_odom_{false};
  nav_msgs::msg::Odometry odom_;
  quad_control::ReferenceBuffer ref_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quad_interfaces::msg::ReferenceTrajectory>::SharedPtr ref_sub_;
  rclcpp::Publisher<quad_interfaces::msg::RatesThrust>::SharedPtr cmd_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionController>());
  rclcpp::shutdown();
  return 0;
}
