#pragma once

#include <array>

#include <Eigen/Dense>

namespace quad_webots
{

struct Rotor
{
  double x, y;   // thrust point relative to the centre of mass, body frame [m]
  int spin;      // +1 if the motor turns positively about body +z, -1 otherwise
};

// Allocation between [T, tau_x, tau_y, tau_z] and the four rotor thrusts f_i.
//   T     = sum f_i
//   tau_x = sum  y_i f_i
//   tau_y = sum -x_i f_i
//   tau_z = sum -spin_i (k_q / k_t) f_i      (reaction torque of the propellers)
// Each rotor follows f_i = k_t * w_i^2, and its speed carries the spin sign.
class Mixer
{
public:
  Mixer(const std::array<Rotor, 4> & rotors, double k_t, double k_q, double max_speed);

  // Rotor speeds [rad/s] for a wrench; thrusts are clipped to [0, f_max].
  std::array<double, 4> speeds(double thrust, const Eigen::Vector3d & torque) const;

  // Wrench produced by given rotor speeds (used by the tests).
  Eigen::Vector4d wrench(const std::array<double, 4> & speeds) const;

  double maxThrustPerRotor() const { return f_max_; }

private:
  std::array<Rotor, 4> rotors_;
  double k_t_, max_speed_, f_max_;
  Eigen::Matrix4d A_, A_inv_;
};

}  // namespace quad_webots
