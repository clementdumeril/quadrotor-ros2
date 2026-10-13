#include <gtest/gtest.h>

#include "quad_webots/mixer.hpp"

using quad_webots::Mixer;
using quad_webots::Rotor;

namespace
{
// Mavic 2 Pro geometry relative to the centre of mass (config/mavic.yaml).
Mixer mavic()
{
  return Mixer({Rotor{0.1428537, 0.151294, 1}, Rotor{0.1428537, -0.151294, -1},
                Rotor{-0.089179, 0.127453, -1}, Rotor{-0.089179, -0.127453, 1}},
               2.6e-4, 5.2e-6, 576.0);
}
}  // namespace

TEST(Mixer, RoundTripReproducesTheWrench)
{
  const Mixer m = mavic();
  const Eigen::Vector3d tau(0.02, -0.03, 0.004);
  const Eigen::Vector4d w = m.wrench(m.speeds(5.0, tau));
  EXPECT_NEAR(w(0), 5.0, 1e-9);
  EXPECT_NEAR(w(1), tau.x(), 1e-9);
  EXPECT_NEAR(w(2), tau.y(), 1e-9);
  EXPECT_NEAR(w(3), tau.z(), 1e-9);
}

TEST(Mixer, HoverHasNoTorqueAndCarriesTheSpinSigns)
{
  const Mixer m = mavic();
  const auto s = m.speeds(4.905, Eigen::Vector3d::Zero());
  EXPECT_GT(s[0], 0.0);
  EXPECT_LT(s[1], 0.0);
  EXPECT_LT(s[2], 0.0);
  EXPECT_GT(s[3], 0.0);
  // The centre of mass sits behind the front rotors' lever arm, so the rear rotors carry more.
  EXPECT_GT(std::abs(s[2]), std::abs(s[0]));
  EXPECT_NEAR(m.wrench(s).tail<3>().norm(), 0.0, 1e-9);
}

TEST(Mixer, ClipsToTheMotorLimits)
{
  const Mixer m = mavic();
  for (double w : m.speeds(1e3, Eigen::Vector3d::Zero())) {
    EXPECT_LE(std::abs(w), 576.0 + 1e-9);
  }
  for (double w : m.speeds(-1.0, Eigen::Vector3d::Zero())) {
    EXPECT_EQ(w, 0.0);
  }
}
