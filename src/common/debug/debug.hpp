#ifndef DEBUG_H
#define DEBUG_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <unitree_legged_msgs/AllLegsInfo.h>
#include <unitree_legged_msgs/BodyInfo.h>
#include <unitree_legged_msgs/StateError.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <cppTypes.h>

// #include <controllers/convexMPC/Metrics.h>

using std::cout;
using std::endl;

// #define PUB_IMU_AND_ODOM

struct MetricData
{
  Vec4<float> final_body_cost; 
  Vec4<float> final_cost;
  Vec4<float> final_leg_cost;
};

class Debug
{
public:
  Debug(ros::Time time_start);

  void updatePlot();
  void updateVisualization();
  void tfPublish();

  unitree_legged_msgs::AllLegsInfo all_legs_info = {};
  unitree_legged_msgs::BodyInfo body_info = {};
  nav_msgs::Odometry ground_truth_odom = {};
  sensor_msgs::Imu imu;
  MetricData metric_data = {};

private:
  void _init();
  void _initPublishers();

  ros::NodeHandle _nh;
  const ros::Time _zero_time;
  ros::Time _time_start;

  ros::Publisher _pub_joint_states;
  ros::Publisher _pub_imu;
  ros::Publisher _pub_all_legs_info;
  ros::Publisher _pub_odom;
  ros::Publisher _pub_body_info;
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster world_broadcaster;
};

#endif //DEBUG_H
