#ifndef ROTORS_DANYLO_LQR_CONTROLLER_HPP_
#define ROTORS_DANYLO_LQR_CONTROLLER_HPP_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_danylo/common.hpp"
#include "rotors_danylo/parameters.hpp"

namespace lqr_control {

class LQRControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW //required macro to have Eigen members
  LQRControllerParameters() {}

  Eigen::Matrix4d allocation_matrix_;
  Eigen::Matrix4d inverse_allocation_matrix_;
  Eigen::Matrix4d inertia_;
  Eigen::Matrix<double, 4, 12> K_lqr_; // LQR gain matrix
};

class LQRController {
 public:
  LQRController();
  ~LQRController();
  
  void InitializeParameters(const ros::NodeHandle& nh);
  void GetGainMatrix(const ros::NodeHandle& nh);
  
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities);

  void SetOdometry(const rotors_danylo::EigenOdometry& odometry);
  void SetIMU(const rotors_danylo::EigenIMU& imu);
  void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);

  LQRControllerParameters controller_parameters_;
  rotors_danylo::VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  bool initialized_params_;
  bool controller_active_;
  
  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  rotors_danylo::EigenOdometry odometry_;
  rotors_danylo::EigenIMU imu_;

  Eigen::Matrix<double, 12, 1> state; // measured state
  Eigen::Matrix<double, 12, 1> state_d; // desired state
};
}

#endif // ROTORS_DANYLO_LQR_CONTROLLER_HPP_
