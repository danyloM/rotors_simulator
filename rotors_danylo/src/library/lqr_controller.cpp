#include <ros/ros.h>

#include "rotors_danylo/library/lqr_controller.hpp"

namespace lqr_control {

LQRController::LQRController() : initialized_params_(false), controller_active_(false) {}

LQRController::~LQRController() {}

void LQRController::InitializeParameters(const ros::NodeHandle& nh) {
  // Compute allocation matrix (map from rotor velocitiyes to U_i generalized force and moments)
  double l = vehicle_parameters_.rotor_configuration_.rotors[0].arm_length; // boom length
  double k_n = vehicle_parameters_.rotor_configuration_.rotors[0].rotor_force_constant; // motor thrust constant
  double k_m = vehicle_parameters_.rotor_configuration_.rotors[0].rotor_moment_constant; // motor torque constant
  
  controller_parameters_.allocation_matrix_ <<
      k_n,      k_n,      k_n,     k_n,
      0,        l*k_n,    0,       -l*k_n,
      -l*k_n,   0,        l*k_n,   0,
      k_m*k_n,  -k_m*k_n, k_m*k_n, -k_m*k_n;

  // We initilize the inertia matrix I here.
  controller_parameters_.inertia_.setZero();
  controller_parameters_.inertia_(0, 0) = vehicle_parameters_.mass_;
  controller_parameters_.inertia_.block<3, 3>(1, 1) = vehicle_parameters_.inertia_;

  // Calculate the inverse allocation matrix (possible to do exactly for quadcopters, unlike hexacopters!)
  controller_parameters_.inverse_allocation_matrix_ = controller_parameters_.allocation_matrix_.inverse();

  // Define the LQR gain matrix
  GetGainMatrix(nh);

  // Finish initialization
  initialized_params_ = true;
}

void LQRController::GetGainMatrix(const ros::NodeHandle& nh) {
  // Load the LQR gain matrix from lqr_controller.yaml file
  std::map<std::string, double > gain_matrix_row;
  double gain_matrix_element;
  std::string gain_matrix_string = "K/";
  unsigned int i = 0;
  unsigned int j = 0;
  while (nh.getParam(gain_matrix_string + std::to_string(i), gain_matrix_row)) {
    j = 0;
    if (i == 4) {
      ROS_ERROR_STREAM("[rosparam]: LQR gain matrix has too many rows, shutting down controller!");
      ros::shutdown();
    } else {
      while (nh.getParam(gain_matrix_string + std::to_string(i) + "/" + std::to_string(j), gain_matrix_element)) {
        if (j == 12) {
          ROS_ERROR_STREAM("[rosparam]: LQR gain matrix has too many columns, shutting down controller!");
          ros::shutdown();
        } else {
          controller_parameters_.K_lqr_(i,j) = gain_matrix_element;
          ++j;
        }
      }
      ++i;
    }
  }
  if ( (i != 4) || (j != 12) ) {
    ROS_ERROR_STREAM("[rosparam]: could not get the full [4 x 12] LQR gain matrix, shutting down controller!");
    ros::shutdown();
  }
}

void LQRController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }
  //////////////////////////////////////////////////////////////////////////////
  // Direct LQR controller using quadcopter model linearized around hover
  //
  // Source: Francesco Sabatino, KTH Master's thesis, "Quadrotor
  // control: modeling, nonlinear control design, and simulation"

  // get Tait-Bryan Euler angles
  Eigen::Vector3d roll_pitch_yaw;
  rotors_danylo::quat2rpy(odometry_.orientation, &roll_pitch_yaw);

  // get measured state
  state << roll_pitch_yaw.x(), roll_pitch_yaw.y(), roll_pitch_yaw.z(),
      odometry_.angular_velocity.x(), odometry_.angular_velocity.y(), odometry_.angular_velocity.z(),
      odometry_.velocity.x(), odometry_.velocity.y(), odometry_.velocity.z(),
      odometry_.position.x(), odometry_.position.y(), odometry_.position.z();
  
  // get desired state
  Eigen::Quaterniond q = odometry_.orientation;
  Eigen::Matrix3d R_B_W; // world to body
  R_B_W << 1-2*(q.y()*q.y()+q.z()*q.z()), 2*(q.x()*q.y()+q.w()*q.z()), 2*(q.x()*q.z()-q.w()*q.y()),
      2*(q.x()*q.y()-q.w()*q.z()), 1-2*(q.x()*q.x()+q.z()*q.z()), 2*(q.y()*q.z()+q.w()*q.x()),
      2*(q.x()*q.z()+q.w()*q.y()), 2*(q.y()*q.z()-q.w()*q.x()), 1-2*(q.x()*q.x()+q.y()*q.y());

  Eigen::Vector3d command_velocity_B = R_B_W*command_trajectory_.velocity_W;
  Eigen::Vector3d command_angular_velocity_B = R_B_W*command_trajectory_.angular_velocity_W;
  
  state_d << 0, 0, command_trajectory_.getYaw(),
      command_angular_velocity_B.x(), command_angular_velocity_B.y(), command_angular_velocity_B.z(),
      command_velocity_B.x(), command_velocity_B.y(), command_velocity_B.z(),
      command_trajectory_.position_W.x(), command_trajectory_.position_W.y(), command_trajectory_.position_W.z();
  
  // multiply by input gain matrix (obtained using LQR)
  Eigen::Vector4d u = -controller_parameters_.K_lqr_*(state-state_d);
  u(0) += vehicle_parameters_.mass_*vehicle_parameters_.gravity_; // account for gravity
  
  // convert to rotor velocity commands
  *rotor_velocities = controller_parameters_.inverse_allocation_matrix_*u;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows())); // min rotor velocity is zero
  *rotor_velocities = rotor_velocities->cwiseSqrt(); // take elemenwise square root, since so far have (velocity)^2
}

void LQRController::SetOdometry(const rotors_danylo::EigenOdometry& odometry) {
  odometry_ = odometry; //now use odometry_.<member name> to get odometry sensor output
}

void LQRController::SetIMU(const rotors_danylo::EigenIMU& imu) {
  imu_ = imu; //now use imu_.<member name> to get imu sensor output
}

void LQRController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

}
