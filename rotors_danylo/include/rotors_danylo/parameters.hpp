/*
 * Original file: rotors_control/include/rotors_control/parameters.h
 *
 * Copied here to have the freedom of modifying this file without breaking
 * something in the rotors_control package.
 *
 * Changelog:
 *   (v1.0) original creation, copy of the original file above
 *
 *   (v1.1) generalized such that can be used for any quadcopter
 *          namely: got rid of default parameters for AscTec Firefly
 */

#ifndef INCLUDE_ROTORS_DANYLO_PARAMETERS_HPP_
#define INCLUDE_ROTORS_DANYLO_PARAMETERS_HPP_

namespace rotors_danylo {
// Default physics parameters.
static constexpr double kDefaultGravity = 9.81;

struct Rotor {
  Rotor()
      : angle(0.0),
        direction(1) {}
  Rotor(double _angle, double _arm_length,
        double _rotor_force_constant, double _rotor_moment_constant,
        int _direction)
      : angle(_angle),
        arm_length(_arm_length),
        rotor_force_constant(_rotor_force_constant),
        rotor_moment_constant(_rotor_moment_constant),
        direction(_direction) {}
  double angle;
  double arm_length;
  double rotor_force_constant;
  double rotor_moment_constant;
  int direction;
};

struct RotorConfiguration {
  std::vector<Rotor> rotors;
};

class VehicleParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VehicleParameters()
      : gravity_(kDefaultGravity) {}
  double mass_;
  const double gravity_;
  Eigen::Matrix3d inertia_;
  RotorConfiguration rotor_configuration_;
};

}

#endif /* INCLUDE_ROTORS_DANYLO_PARAMETERS_HPP_ */
