#include <cmath>

#include <gtest/gtest.h>

#include "quad_control/reference.hpp"

using quad_control::attitudeFromAcceleration;
using quad_control::kGravity;
using quad_control::ReferenceBuffer;

namespace
{
quad_interfaces::msg::ReferenceTrajectory makeLine(double t0, double dt, int n, double speed)
{
  quad_interfaces::msg::ReferenceTrajectory m;
  m.header.stamp.sec = static_cast<int32_t>(t0);
  m.header.stamp.nanosec = static_cast<uint32_t>((t0 - std::floor(t0)) * 1e9);
  m.dt = dt;
  for (int k = 0; k < n; ++k) {
    geometry_msgs::msg::Point p;
    p.x = speed * k * dt;
    geometry_msgs::msg::Vector3 v;
    v.x = speed;
    m.position.push_back(p);
    m.velocity.push_back(v);
    m.acceleration.push_back(geometry_msgs::msg::Vector3());
    m.yaw.push_back(0.0);
  }
  return m;
}
}  // namespace

TEST(ReferenceBuffer, InterpolatesBetweenSamples)
{
  ReferenceBuffer ref;
  ref.set(makeLine(10.0, 0.02, 51, 2.0));
  ASSERT_TRUE(ref.valid());
  EXPECT_NEAR(ref.at(10.0).p.x(), 0.0, 1e-9);
  EXPECT_NEAR(ref.at(10.25).p.x(), 0.5, 1e-9);   // between samples 12 and 13
  EXPECT_NEAR(ref.at(10.25).v.x(), 2.0, 1e-9);
}

TEST(ReferenceBuffer, ClampsOutsideTheSamples)
{
  ReferenceBuffer ref;
  ref.set(makeLine(10.0, 0.02, 51, 2.0));
  EXPECT_NEAR(ref.at(9.0).p.x(), 0.0, 1e-9);     // before: first sample
  const auto end = ref.at(20.0);                 // after: last point, at rest
  EXPECT_NEAR(end.p.x(), 2.0, 1e-9);
  EXPECT_NEAR(end.v.norm(), 0.0, 1e-12);
}

TEST(ReferenceBuffer, IgnoresMalformedMessages)
{
  ReferenceBuffer ref;
  auto m = makeLine(0.0, 0.02, 5, 1.0);
  m.yaw.pop_back();
  ref.set(m);
  EXPECT_FALSE(ref.valid());
}

TEST(Attitude, HoverIsLevelWithTheRequestedYaw)
{
  const Eigen::Matrix3d R = attitudeFromAcceleration(Eigen::Vector3d(0, 0, kGravity), M_PI / 2);
  EXPECT_TRUE(R.col(2).isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_TRUE(R.col(0).isApprox(Eigen::Vector3d::UnitY(), 1e-12));
}

TEST(Attitude, ThrustAxisFollowsTheAcceleration)
{
  const Eigen::Vector3d acc(3.0, -1.0, kGravity);
  const Eigen::Matrix3d R = attitudeFromAcceleration(acc, 0.3);
  EXPECT_TRUE(R.col(2).isApprox(acc.normalized(), 1e-12));
  EXPECT_NEAR((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
}
