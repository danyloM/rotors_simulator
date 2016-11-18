/*
 * Original file: rotors_control/include/rotors_control/parameters_ros.h
 *
 * Copied here to have the freedom of modifying this file without breaking
 * something in the rotors_control package.
 *
 * Changelog:
 *   (v1.0) original creation, copy of the original file above
 *
 *   (v1.1) removed default value option for GetRosParameter() such
 *          that now the controller requires a valid <quad name>.yaml
 *          file to be loaded. This enables this header and
 *          parameters.hpp to be used with any quad.
 */

#ifndef INCLUDE_ROTORS_DANYLO_PARAMETERS_ROS_HPP_
#define INCLUDE_ROTORS_DANYLO_PARAMETERS_ROS_HPP_

#include <ros/ros.h>

#include "rotors_danylo/parameters.hpp"

namespace rotors_danylo {

template<typename T> inline void GetRosParameter(const ros::NodeHandle& nh,
                                                 const std::string& key,
                                                 T* value) {
  ROS_ASSERT(value != nullptr);
  bool have_parameter = nh.getParam(key, *value);
  if (!have_parameter) {
    ROS_ERROR_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                    << "/" << key << ", shutting down controller!");
    ros::shutdown();
  }
}

inline void GetRotorConfiguration(const ros::NodeHandle& nh,
                                  RotorConfiguration* rotor_configuration) {
  std::map<std::string, double> single_rotor;
  std::string rotor_configuration_string = "rotor_configuration/";
  unsigned int i = 0;
  while (nh.getParam(rotor_configuration_string + std::to_string(i), single_rotor)) {
    if (i == 0) {
      rotor_configuration->rotors.clear();
    }
    Rotor rotor;
    nh.getParam(rotor_configuration_string + std::to_string(i) + "/angle",
                 rotor.angle);
    nh.getParam(rotor_configuration_string + std::to_string(i) + "/arm_length",
                 rotor.arm_length);
    nh.getParam(rotor_configuration_string + std::to_string(i) + "/rotor_force_constant",
                 rotor.rotor_force_constant);
    nh.getParam(rotor_configuration_string + std::to_string(i) + "/rotor_moment_constant",
                 rotor.rotor_moment_constant);
    nh.getParam(rotor_configuration_string + std::to_string(i) + "/direction",
                 rotor.direction);
    rotor_configuration->rotors.push_back(rotor);
    ++i;
  }
  if (i == 0) {
    ROS_ERROR_STREAM("[rosparam]: could not find any rotor_configuration parameters, shutting down controller!");
    ros::shutdown();
  }
}

inline void GetVehicleParameters(const ros::NodeHandle& nh, VehicleParameters* vehicle_parameters) {
  GetRosParameter(nh, "mass",&vehicle_parameters->mass_);
  GetRosParameter(nh, "inertia/xx",&vehicle_parameters->inertia_(0, 0));
  GetRosParameter(nh, "inertia/xy",&vehicle_parameters->inertia_(0, 1));
  vehicle_parameters->inertia_(1, 0) = vehicle_parameters->inertia_(0, 1);
  GetRosParameter(nh, "inertia/xz",&vehicle_parameters->inertia_(0, 2));
  vehicle_parameters->inertia_(2, 0) = vehicle_parameters->inertia_(0, 2);
  GetRosParameter(nh, "inertia/yy",&vehicle_parameters->inertia_(1, 1));
  GetRosParameter(nh, "inertia/yz",&vehicle_parameters->inertia_(1, 2));
  vehicle_parameters->inertia_(2, 1) = vehicle_parameters->inertia_(1, 2);
  GetRosParameter(nh, "inertia/zz",&vehicle_parameters->inertia_(2, 2));
  GetRotorConfiguration(nh, &vehicle_parameters->rotor_configuration_);
}
}

#endif // INCLUDE_ROTORS_DANYLO_PARAMETERS_ROS_HPP_
