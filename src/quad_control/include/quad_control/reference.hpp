// Shared by the controllers: the reference trajectory, interpolated at any time, and the
// attitude that produces a given acceleration (differential flatness of the quadrotor).
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/time.hpp>

#include "quad_interfaces/msg/reference_trajectory.hpp"

namespace quad_control
{

constexpr double kGravity = 9.81;

struct RefPoint
{
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

inline double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// Holds the latest /quad/reference message. Times before the first sample give the first
// sample, times after the last give the last one with zero velocity and acceleration
// (the drone then holds that point).
class ReferenceBuffer
{
public:
  void set(const quad_interfaces::msg::ReferenceTrajectory & msg)
  {
    const size_t n = msg.position.size();
    if (n == 0 || msg.velocity.size() != n || msg.acceleration.size() != n || msg.yaw.size() != n ||
        (n > 1 && msg.dt <= 0.0)) {
      return;
    }
    t0_ = rclcpp::Time(msg.header.stamp).seconds();
    dt_ = msg.dt;
    pts_.resize(n);
    for (size_t k = 0; k < n; ++k) {
      const auto & p = msg.position[k];
      const auto & v = msg.velocity[k];
      const auto & a = msg.acceleration[k];
      pts_[k].p = {p.x, p.y, p.z};
      pts_[k].v = {v.x, v.y, v.z};
      pts_[k].a = {a.x, a.y, a.z};
      pts_[k].yaw = msg.yaw[k];
    }
  }

  bool valid() const { return !pts_.empty(); }

  RefPoint at(double t) const
  {
    if (pts_.size() == 1 || t <= t0_) {
      return pts_.front();
    }
    const double s = (t - t0_) / dt_;
    const size_t k = static_cast<size_t>(s);
    if (k + 1 >= pts_.size()) {
      RefPoint end = pts_.back();
      end.v.setZero();
      end.a.setZero();
      return end;
    }
    const double f = s - static_cast<double>(k);
    const RefPoint & A = pts_[k];
    const RefPoint & B = pts_[k + 1];
    RefPoint r;
    r.p = A.p + f * (B.p - A.p);
    r.v = A.v + f * (B.v - A.v);
    r.a = A.a + f * (B.a - A.a);
    r.yaw = wrapAngle(A.yaw + f * wrapAngle(B.yaw - A.yaw));
    return r;
  }

private:
  double t0_{0.0}, dt_{0.0};
  std::vector<RefPoint> pts_;
};

// Rotation whose body z axis is along acc (thrust direction, gravity included) and whose
// heading is yaw: z_b = acc/|acc|, y_b = z_b x x_c, x_b = y_b x z_b (Mellinger & Kumar 2011).
inline Eigen::Matrix3d attitudeFromAcceleration(const Eigen::Vector3d & acc, double yaw)
{
  const Eigen::Vector3d zb = acc.norm() > 1e-6 ? acc.normalized() : Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d xc(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d yb = zb.cross(xc);
  yb = yb.norm() > 1e-6 ? yb.normalized() : Eigen::Vector3d::UnitY();
  Eigen::Matrix3d R;
  R.col(0) = yb.cross(zb);
  R.col(1) = yb;
  R.col(2) = zb;
  return R;
}

}  // namespace quad_control
