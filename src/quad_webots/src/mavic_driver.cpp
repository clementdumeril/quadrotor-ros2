#include "quad_webots/mavic_driver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

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
  // The Webots Propeller model is not used: its thrust followed the motor commands with a lag of
  // several seconds in our tests (scripts/motor_ident.py), which no controller can handle and
  // which is not documented. The rotors are modelled here instead (first-order motor, f = k_t w^2,
  // reaction torque k_q w^2) and their forces are applied through the supervisor API. The
  // propeller motors stay at zero speed, so Webots adds no thrust of its own.
  for (int i = 0; i < 4; ++i) {
    motors_[i] = wb_robot_get_device(kMotorNames[i]);
    wb_motor_set_position(motors_[i], INFINITY);
    wb_motor_set_velocity(motors_[i], 0.0);
  }
  if (!wb_robot_get_supervisor()) {
    RCLCPP_FATAL(node_->get_logger(), "The robot must be a supervisor (world: supervisor TRUE) to apply rotor forces");
    throw std::runtime_error("robot is not a supervisor");
  }
  self_ = wb_supervisor_node_get_self();

  // Physical constants come from the Mavic2Pro PROTO (Webots R2025a); see config/mavic.yaml.
  mass_ = node_->declare_parameter("mass", 0.5);
  hover_thrust_ = mass_ * kGravity;
  max_motor_speed_ = node_->declare_parameter("max_motor_speed", 576.0);
  cmd_timeout_ = node_->declare_parameter("cmd_timeout", 0.3);
  const double k_t = node_->declare_parameter("k_thrust", 2.6e-4);
  const double k_q = node_->declare_parameter("k_torque", 5.2e-6);
  const auto rx = node_->declare_parameter("rotor_x", std::vector<double>{0.116, 0.116, -0.116, -0.116});
  const auto ry = node_->declare_parameter("rotor_y", std::vector<double>{0.151, -0.151, 0.127, -0.127});
  const auto spin = node_->declare_parameter("rotor_spin", std::vector<int64_t>{1, -1, -1, 1});
  // Thrust points in the robot frame (Propeller.centerOfThrust), where the forces are applied.
  const auto ox = node_->declare_parameter("thrust_point_x", std::vector<double>{0.0549, 0.0549, -0.1772, -0.1772});
  const auto oy = node_->declare_parameter("thrust_point_y", std::vector<double>{0.1513, -0.1513, 0.1275, -0.1275});
  const auto oz = node_->declare_parameter("thrust_point_z", std::vector<double>{-0.0028, -0.0028, -0.0320, -0.0320});
  motor_tau_ = node_->declare_parameter("motor_time_constant", 0.03);
  k_t_ = k_t;
  k_q_ = k_q;
  kp_rate_ = toEigen(node_->declare_parameter("kp_rate", std::vector<double>{0.05, 0.05, 0.02}));
  ki_rate_ = toEigen(node_->declare_parameter("ki_rate", std::vector<double>{0.1, 0.1, 0.02}));
  i_limit_ = toEigen(node_->declare_parameter("rate_integral_limit", std::vector<double>{0.05, 0.05, 0.02}));

  std::array<Rotor, 4> rotors{};
  for (int i = 0; i < 4; ++i) {
    rotors[i] = Rotor{rx.at(i), ry.at(i), static_cast<int>(spin.at(i))};
    thrust_points_[i] = {ox.at(i), oy.at(i), oz.at(i)};
    spin_[i] = static_cast<int>(spin.at(i));
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

  raw_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/quad/cmd/motor_speeds_raw", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      for (size_t i = 0; i < 4 && i < msg->data.size(); ++i) {
        raw_speeds_[i] = std::clamp(msg->data[i], -max_motor_speed_, max_motor_speed_);
      }
      last_raw_time_ = node_->get_clock()->now();
      raw_received_ = true;
    });

  disturbance_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
    "/quad/disturbance/force", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr m) {
      disturbance_ = Eigen::Vector3d(m->x, m->y, m->z);
    });
  // A path on the Webots host (e.g. C:/Users/me/flight.mp4) starts recording; an empty string stops it.
  record_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/quad/record", 10, [this](const std_msgs::msg::String::SharedPtr m) {
      if (!m->data.empty() && !recording_) {
        startRecording(m->data);
      } else if (m->data.empty() && recording_) {
        wb_supervisor_movie_stop_recording();
        recording_ = false;
        RCLCPP_INFO(node_->get_logger(), "Recording stopped");
      }
    });

  setpoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/quad/setpoint", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
      setpoint_ = Eigen::Vector3d(m->pose.position.x, m->pose.position.y, m->pose.position.z);
    });
  view_position_ = node_->declare_parameter("record_view_position", std::vector<double>{0.0, -6.2, 3.6});
  view_target_ = node_->declare_parameter("record_view_target", std::vector<double>{0.0, 0.3, 1.2});

  RCLCPP_INFO(node_->get_logger(),
              "Mavic driver ready: step %.1f ms, hover thrust %.2f N, max %.2f N, motor time constant %.0f ms",
              dt_ * 1e3, mass_ * kGravity, 4.0 * mixer_->maxThrustPerRotor(), motor_tau_ * 1e3);
}

void MavicDriver::startRecording(const std::string & file)
{
  // Fixed overview camera looking at the flight area (the world's camera follows the drone).
  WbFieldRef children = wb_supervisor_node_get_field(wb_supervisor_node_get_root(), "children");
  for (int i = 0; i < wb_supervisor_field_get_count(children); ++i) {
    WbNodeRef n = wb_supervisor_field_get_mf_node(children, i);
    if (wb_supervisor_node_get_type(n) != WB_NODE_VIEWPOINT) {
      continue;
    }
    const Eigen::Vector3d pos(view_position_.at(0), view_position_.at(1), view_position_.at(2));
    const Eigen::Vector3d target(view_target_.at(0), view_target_.at(1), view_target_.at(2));
    Eigen::Matrix3d R;  // Webots viewpoint: looks along its x axis, z up
    R.col(0) = (target - pos).normalized();
    R.col(1) = Eigen::Vector3d::UnitZ().cross(R.col(0)).normalized();
    R.col(2) = R.col(0).cross(R.col(1));
    const Eigen::AngleAxisd aa(R);
    const double position[3] = {pos.x(), pos.y(), pos.z()};
    const double rotation[4] = {aa.axis().x(), aa.axis().y(), aa.axis().z(), aa.angle()};
    wb_supervisor_field_set_sf_string(wb_supervisor_node_get_field(n, "follow"), "");
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(n, "position"), position);
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(n, "orientation"), rotation);
  }
  // Two paths, refilled from scratch at each recording. Webots draws lines one pixel wide, so
  // each path is kTrailLayers polylines a few millimetres apart, which reads as a thicker line.
  const char * colors[2] = {"0.15 0.15 0.15", "0.16 0.47 0.84"};
  for (int k = 0; k < 2 * kTrailLayers; ++k) {
    if (trail_nodes_[k]) {
      wb_supervisor_node_remove(trail_nodes_[k]);
    }
    const std::string def = "QUAD_TRAIL_" + std::to_string(k);
    const char * color = colors[k / kTrailLayers];
    const std::string shape = "DEF " + def + " Shape { appearance Appearance { material Material { diffuseColor " +
                              color + " emissiveColor " + color +
                              " } } geometry IndexedLineSet { coord Coordinate { point [ ] } coordIndex [ ] } "
                              "castShadows FALSE }";
    wb_supervisor_field_import_mf_node_from_string(children, -1, shape.c_str());
    trail_nodes_[k] = wb_supervisor_node_get_from_def(def.c_str());
    WbNodeRef geometry = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(trail_nodes_[k], "geometry"));
    WbNodeRef coord = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(geometry, "coord"));
    trail_points_[k] = wb_supervisor_node_get_field(coord, "point");
    trail_index_[k] = wb_supervisor_node_get_field(geometry, "coordIndex");
  }
  trail_count_ = 0;
  wb_supervisor_movie_start_recording(file.c_str(), 1280, 720, 0, 90, 1, false);
  recording_ = true;
  RCLCPP_INFO(node_->get_logger(), "Recording the 3D view to %s", file.c_str());
}

void MavicDriver::updateTrails()
{
  // One point every 5 steps (20 ms); the reference trail only once a setpoint has arrived.
  if (!recording_ || ++trail_tick_ % 5 != 0 || !trail_points_[0] || std::isnan(setpoint_.x())) {
    return;
  }
  for (int k = 0; k < 2 * kTrailLayers; ++k) {
    const Eigen::Vector3d c = (k < kTrailLayers ? setpoint_ : p_) +
                              Eigen::Vector3d(0.0, 0.0, 0.004 * (k % kTrailLayers - kTrailLayers / 2));
    const double point[3] = {c.x(), c.y(), c.z()};
    wb_supervisor_field_insert_mf_vec3f(trail_points_[k], -1, point);
    wb_supervisor_field_insert_mf_int32(trail_index_[k], -1, trail_count_);
  }
  ++trail_count_;
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
  // No integration while the feet touch the ground: the error there is not a model error
  // and the stored torque would be released at lift-off.
  if (p_.z() > ground_z_ + 0.05) {
    rate_integral_ = (rate_integral_ + ki_rate_.cwiseProduct(err) * dt).cwiseMax(-i_limit_).cwiseMin(i_limit_);
  }
  return kp_rate_.cwiseProduct(err) + rate_integral_;
}

void MavicDriver::logCenterOfMass()
{
  // Identification helper, only when the robot is a supervisor (world: supervisor TRUE):
  // centre of mass of the whole robot (body, propellers, gimbal) in the body frame.
  if (!wb_robot_get_supervisor()) {
    return;
  }
  WbNodeRef self = wb_supervisor_node_get_self();
  const double * c = wb_supervisor_node_get_center_of_mass(self);
  const double * p = wb_supervisor_node_get_position(self);
  const double * R = wb_supervisor_node_get_orientation(self);  // row-major 3x3
  const Eigen::Matrix3d Rm = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R);
  const Eigen::Vector3d com = Rm.transpose() * (Eigen::Vector3d(c[0], c[1], c[2]) - Eigen::Vector3d(p[0], p[1], p[2]));
  RCLCPP_INFO(node_->get_logger(), "Centre of mass in the body frame: [%.4f, %.4f, %.4f] m",
              com.x(), com.y(), com.z());

}

void MavicDriver::applyRotorForces(const std::array<double, 4> & commanded)
{
  // First-order motor: w <- w + (w_cmd - w)(1 - exp(-dt/tau)); then thrust and reaction torque.
  const double a = 1.0 - std::exp(-dt_ / motor_tau_);
  double yaw_torque = 0.0;
  for (int i = 0; i < 4; ++i) {
    rotor_speed_[i] += a * (commanded[i] - rotor_speed_[i]);
    const double w2 = rotor_speed_[i] * rotor_speed_[i];
    const double force[3] = {0.0, 0.0, k_t_ * w2};
    const double offset[3] = {thrust_points_[i][0], thrust_points_[i][1], thrust_points_[i][2]};
    wb_supervisor_node_add_force_with_offset(self_, force, offset, true);
    yaw_torque += -spin_[i] * k_q_ * w2;
  }
  const double torque[3] = {0.0, 0.0, yaw_torque};
  wb_supervisor_node_add_torque(self_, torque, true);
  if (disturbance_.squaredNorm() > 0.0) {  // test input, world frame, at the centre of mass
    const double f[3] = {disturbance_.x(), disturbance_.y(), disturbance_.z()};
    wb_supervisor_node_add_force(self_, f, false);
  }
}

void MavicDriver::step()
{
  if (!com_logged_) {
    logCenterOfMass();
    com_logged_ = true;
  }
  readState();
  if (std::isnan(ground_z_)) {
    ground_z_ = p_.z();
  }
  const rclcpp::Time now = node_->get_clock()->now();
  // Measurements are stamped with the Webots time of this step. The node clock follows /clock,
  // which can arrive a step late: then two consecutive samples shared a stamp, a 4 ms error
  // that is 2 cm at 5 m/s and corrupts finite differences of the velocity.
  publishState(rclcpp::Time(static_cast<int64_t>(std::llround(wb_robot_get_time() * 1e9)), RCL_ROS_TIME));
  updateTrails();

  std::array<double, 4> speeds{0.0, 0.0, 0.0, 0.0};
  if (armed_) {
    double thrust = cmd_.thrust;
    Eigen::Vector3d rates_des(cmd_.body_rates.x, cmd_.body_rates.y, cmd_.body_rates.z);
    if ((now - last_cmd_time_).seconds() > cmd_timeout_) {
      // Failsafe: level the body and descend at 0.4 m/s, using the hover thrust learned in flight
      // (so a wrong mass parameter cannot make the drone climb).
      const Eigen::Vector3d z_body = q_ * Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d tilt = q_.conjugate() * z_body.cross(Eigen::Vector3d::UnitZ());
      rates_des = Eigen::Vector3d(4.0 * tilt.x(), 4.0 * tilt.y(), 0.0);
      thrust = hover_thrust_ / kGravity * (kGravity + 2.0 * (-0.4 - v_world_.z()));
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "No command for %.2f s: failsafe descent", (now - last_cmd_time_).seconds());
      if (p_.z() < ground_z_ + 0.03 && std::abs(v_world_.z()) < 0.2) {
        armed_ = false;  // on the ground: stop the motors until the next command
        RCLCPP_INFO(node_->get_logger(), "Landed, motors off");
      }
    } else if (p_.z() > ground_z_ + 0.2 && std::abs(v_world_.z()) < 0.2 && omega_.norm() < 0.3) {
      // Steady flight: low-pass the commanded thrust as the hover-thrust estimate.
      hover_thrust_ += 0.002 * (cmd_.thrust - hover_thrust_);
    }
    if (armed_) {
      speeds = mixer_->speeds(thrust, rateLoop(rates_des, dt_));
    }
  } else {
    rate_integral_.setZero();
  }
  if (raw_received_ && (now - last_raw_time_).seconds() < cmd_timeout_) {
    speeds = raw_speeds_;
  }

  applyRotorForces(speeds);
  std_msgs::msg::Float64MultiArray m;
  m.data.assign(rotor_speed_.begin(), rotor_speed_.end());
  motor_pub_->publish(m);
}

}  // namespace quad_webots

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(quad_webots::MavicDriver, webots_ros2_driver::PluginInterface)
