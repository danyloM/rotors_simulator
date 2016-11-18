/*
 * Danylo Malyuta, ETHZ Zurich, Switzerland
 *
 * ROS node which can subscribe to any topic and which publishes the
 * frequency at which it receives messages from this topic.
 *
 * Changelog:
 *
 *   (v1.0)   Original creation
 *
 */

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>

#define MAKE_SUBSCRIPTION(callback_func) nh.subscribe(topic_name_.c_str(), 1, &TopicProbe::callback_func, this)

namespace frequency_probe {

class TopicProbe {
 public:
  TopicProbe(char topic_name[], char topic_type[]);
  ~TopicProbe();

  void OdometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void IMUCallback(const sensor_msgs::ImuConstPtr& msg);
  void PoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void TrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
  void MotorSpeedCallback(const mav_msgs::ActuatorsConstPtr& msg);
  
  void FrequencyCallback(const ros::TimerEvent& event);
  
 private:
  int num_msgs_received_;
  std::string topic_name_;
  std::string topic_type_;

  // controller input subscribers
  ros::Subscriber odometry_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber trajectory_sub_;
  // controller output subscribers
  ros::Subscriber motor_speed_sub_;

  ros::Publisher frequency_pub_;
  std_msgs::Float32 frequency_msg_;

  ros::Timer calc_timer_;

  double time_now_;
  double time_prev_;
};

TopicProbe::TopicProbe(char topic_name[],
                       char topic_type[]) : num_msgs_received_(0),
                                            topic_name_(topic_name),
                                            topic_type_(topic_type),
                                            time_now_(0),
                                            time_prev_(0) {
  ros::NodeHandle nh;

  // subscribe now to all the topics which the topic probe can possibly monitor the frequency of
  // depending on topic_name_, only one of these subscribtions will actually fire
  ROS_INFO("Subscribing to topic %s",topic_name_.c_str());
  if (topic_type_.compare("odometry") == 0)
  {
    odometry_sub_ = MAKE_SUBSCRIPTION(OdometryCallback);
  }
  else if (topic_type_.compare("imu") == 0 )
  {
    imu_sub_ = MAKE_SUBSCRIPTION(IMUCallback);
  }
  else if (topic_type_.compare("pose") == 0 )
  {
    pose_sub_ = MAKE_SUBSCRIPTION(PoseCallback);
  }
  else if (topic_type_.compare("trajectory") == 0 )
  {
    trajectory_sub_ = MAKE_SUBSCRIPTION(TrajectoryCallback);
  }
  else if (topic_type_.compare("motor_speed") == 0 )
  {
    motor_speed_sub_ = MAKE_SUBSCRIPTION(MotorSpeedCallback);
  }

  // publish the frequency measurement on topic_name_/freq topic
  std::string publish_topic = "/freq_probe/" + topic_type_;
  ROS_INFO("Publishing on topic %s",publish_topic.c_str());
  frequency_pub_ = nh.advertise<std_msgs::Float32>(publish_topic.c_str(), 1);

  // frequency measurement function
  calc_timer_ = nh.createTimer(ros::Duration(1), &TopicProbe::FrequencyCallback, this);
}

TopicProbe::~TopicProbe() {}

void TopicProbe::OdometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  ++num_msgs_received_;
}

void TopicProbe::IMUCallback(const sensor_msgs::ImuConstPtr &msg) {
  ++num_msgs_received_;
}

void TopicProbe::PoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  ++num_msgs_received_;
}

void TopicProbe::TrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  ++num_msgs_received_;
}

void TopicProbe::MotorSpeedCallback(const mav_msgs::ActuatorsConstPtr& msg) {
  ++num_msgs_received_;
}

void TopicProbe::FrequencyCallback(const ros::TimerEvent &event) {
  time_now_ = ros::Time::now().toNSec()*pow(10,-9);
  double dt = time_now_-time_prev_;
  time_prev_ = time_now_;

  // ROS_INFO("Number of messages received: %d, time interval: %.4f",num_msgs_received_,dt);
  frequency_msg_.data = double(num_msgs_received_)/dt;
  frequency_pub_.publish(frequency_msg_);
  
  num_msgs_received_ = 0;
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "frequency_probe_node");

  ros::NodeHandle nh;

  frequency_probe::TopicProbe my_topic_probe(argv[1],argv[2]);

  ros::spin();

  return 0;
}
