#ifndef ROTORS_DANYLO_LQR_CONTROLLER_NODE_HPP_
#define ROTORS_DANYLO_LQR_CONTROLLER_NODE_HPP_

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_danylo/common.hpp"
#include "rotors_danylo/library/lqr_controller.hpp"

namespace lqr_control {

class LQRControllerNode {
 public:
  LQRControllerNode();
  ~LQRControllerNode();

  void InitializeParams();
  void Publish();

 private:

  LQRController lqr_controller_;

  bool used_message_;
  double time_now_;
  double time_prev_;
  int num_msgs_received_;
  
  double controller_Ts_;
  rotors_danylo::EigenOdometry odometry_;
  rotors_danylo::EigenIMU imu_;
  Eigen::VectorXd ref_rotor_velocities_;

  // sensor topics
  std::string ground_truth_;
  std::string odometry_sensor_;

  // subscribers (to receive signals for control)
  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
  ros::Subscriber cmd_pose_sub_;
  ros::Subscriber odometry_sub_;

  // publishers (to publish controller output)
  ros::Publisher motor_velocity_reference_pub_;

  mav_msgs::EigenTrajectoryPointDeque commands_;
  std::deque<ros::Duration> command_waiting_times_;
  ros::Timer command_timer_;
  ros::Timer controller_timer_;

  void TimedCommandCallback(const ros::TimerEvent& e);
  void MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
  void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  void ControllerCallback(const ros::TimerEvent& event);
  
  void TimerCallback(const ros::TimerEvent& event);
  ros::Timer timer_callback_;
};
}

#endif // ROTORS_DANYLO_LQR_CONTROLLER_NODE_HPP_
