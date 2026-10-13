#include "quad_webots/mixer.hpp"

#include <algorithm>
#include <cmath>

namespace quad_webots
{

Mixer::Mixer(const std::array<Rotor, 4> & rotors, double k_t, double k_q, double max_speed)
: rotors_(rotors), k_t_(k_t), max_speed_(max_speed), f_max_(k_t * max_speed * max_speed)
{
  for (int i = 0; i < 4; ++i) {
    A_(0, i) = 1.0;
    A_(1, i) = rotors_[i].y;
    A_(2, i) = -rotors_[i].x;
    A_(3, i) = -rotors_[i].spin * k_q / k_t;
  }
  A_inv_ = A_.inverse();
}

std::array<double, 4> Mixer::speeds(double thrust, const Eigen::Vector3d & torque) const
{
  const Eigen::Vector4d f = A_inv_ * Eigen::Vector4d(thrust, torque.x(), torque.y(), torque.z());
  std::array<double, 4> w{};
  for (int i = 0; i < 4; ++i) {
    const double fi = std::clamp(f(i), 0.0, f_max_);
    w[i] = rotors_[i].spin * std::min(std::sqrt(fi / k_t_), max_speed_);
  }
  return w;
}

Eigen::Vector4d Mixer::wrench(const std::array<double, 4> & speeds) const
{
  Eigen::Vector4d f;
  for (int i = 0; i < 4; ++i) {
    f(i) = k_t_ * speeds[i] * speeds[i];
  }
  return A_ * f;
}

}  // namespace quad_webots
