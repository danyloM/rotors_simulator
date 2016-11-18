#include <string>

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "rotors_danylo/nodes/lqr_controller_node.hpp"
#include "rotors_danylo/parameters_ros.hpp"
#include "rotors_danylo/common.hpp"

namespace lqr_control {

LQRControllerNode::LQRControllerNode() {
  InitializeParams();

  ros::NodeHandle nh;

  cmd_pose_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &LQRControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &LQRControllerNode::MultiDofJointTrajectoryCallback, this);
  
  odometry_sub_ = nh.subscribe((ground_truth_+odometry_sensor_).c_str(), 1,
                               &LQRControllerNode::OdometryCallback, this);

  // motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
  //     mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  // controller_timer_ = nh.createTimer(ros::Duration(controller_Ts_), &LQRControllerNode::ControllerCallback, this);

  timer_callback_ = nh.createTimer(ros::Duration(1), &LQRControllerNode::TimerCallback, this);
  
  command_timer_ = nh.createTimer(ros::Duration(0), &LQRControllerNode::TimedCommandCallback, this,
                                  true, false); //it's a one-shot timer
}

LQRControllerNode::~LQRControllerNode() {}

void LQRControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // sensor topics
  ground_truth_ = "ground_truth/";
  odometry_sensor_ = mav_msgs::default_topics::ODOMETRY;

  num_msgs_received_ = 0;

  // controller execution frequency
  controller_Ts_ = double(1)/100;
  ROS_INFO("Setting controller sampling period to %.5f",controller_Ts_);

  used_message_ = true;
  
  // initialize vehicle parameters
  rotors_danylo::GetVehicleParameters(pnh, &lqr_controller_.vehicle_parameters_);

  // initialize the controller
  lqr_controller_.InitializeParameters(pnh);
}

void LQRControllerNode::Publish() {}

void LQRControllerNode::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  //pose_msg is the most recent pose command to be processed, save it into the EigenTrajectoryPoint deque
  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference); //deque contains 1 element now

  lqr_controller_.SetTrajectoryPoint(commands_.front()); //use this element as the reference signal for the controller
  commands_.pop_front(); //remove this pose command (it's now been passed to controller, don't need it anymore)
}

void LQRControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  lqr_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) { //reuse one-shot timer
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void LQRControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  lqr_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void LQRControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
  ++num_msgs_received_;
  
  // ROS_INFO_ONCE("LQRController got first odometry message.");

  // rotors_danylo::eigenOdometryFromMsg(odometry_msg, &odometry_);
  // lqr_controller_.SetOdometry(odometry_);
}

void LQRControllerNode::TimerCallback(const ros::TimerEvent& event) {
  time_now_ = ros::Time::now().toNSec()*pow(10,-9);
  double dt = time_now_-time_prev_;
  time_prev_ = time_now_;

  printf("t = %.3f freq = %.3f\n",ros::Time::now().toSec(),double(num_msgs_received_)/dt);
  
  num_msgs_received_ = 0;
}

// void LQRControllerNode::ControllerCallback(const ros::TimerEvent& event) {
//   lqr_controller_.CalculateRotorVelocities(&ref_rotor_velocities_);

//   // Todo(ffurrer): Do this in the conversions header.
//   mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

//   actuator_msg->angular_velocities.clear();
//   for (int i = 0; i < ref_rotor_velocities_.size(); i++) {
//     actuator_msg->angular_velocities.push_back(ref_rotor_velocities_[i]);
//   }

//   motor_velocity_reference_pub_.publish(actuator_msg);

//   // time_now_ = ros::Time::now();
//   // printf("time difference: %.4f\n", 1/double((time_now_.toNSec()-time_prev_.toNSec())*pow(10,-9)));
//   // time_prev_ = time_now_;
// }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lqr_controller_node");

  lqr_control::LQRControllerNode lqr_controller_node;

  ros::spin(); // will exit upon pressing Ctrl-C or ros::shutdown()

  if (ros::isShuttingDown()) {
    ROS_ERROR_STREAM("Controller being abnormally shut down (see error messages above), bye bye!");
    return 1;
  } else {
    return 0;
  }
}
