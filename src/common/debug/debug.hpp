#pragma once

#include <Utilities/utilities.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <unitree_legged_msgs/AllLegsInfo.h>
#include <unitree_legged_msgs/BodyInfo.h>
#include <unitree_legged_msgs/StateError.h>
#include <visualization_msgs/Marker.h>

using std::cout;
using std::endl;

// #define PUB_IMU_AND_ODOM

class Debug
{
public:
  Debug(ros::Time time_start);

  void updatePlot();
  void updateVisualization();
  void tfPublish();

  unitree_legged_msgs::AllLegsInfo all_legs_info = {};
  unitree_legged_msgs::BodyInfo body_info = {};
  float z_offset;
  nav_msgs::Odometry ground_truth_odom = {};
  sensor_msgs::Imu imu;
  geometry_msgs::Point last_p_stance[4] = {};
  geometry_msgs::Point last_p_local_stance[4] = {};
  geometry_msgs::PoseWithCovarianceStamped _ground_trurh_pose;

  nav_msgs::Path leg_traj_des[4];
  geometry_msgs::Point leg_force[4];

private:
  void _init();
  void _initPublishers();
  void _ground_truth_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  Vec3<float> _getHipLocation(uint8_t leg_num);
  void _drawLastStancePoints();
  void _drawEstimatedStancePLane();
  void _drawLegsDesiredTrajectory();
  void _drawLegsForce();

  ros::NodeHandle _nh;
  const ros::Time _zero_time;
  ros::Time _time_start;

  ros::Subscriber _sub_ground_truth;

  ros::Publisher _pub_joint_states;
  ros::Publisher _pub_imu;
  ros::Publisher _pub_all_legs_info;
  ros::Publisher _pub_odom;
  ros::Publisher _pub_body_info;
  ros::Publisher _pub_vis_last_p_stance;
  ros::Publisher _pub_vis_estimated_stance_plane;
  ros::Publisher _pub_vis_leg_des_traj[4];
  ros::Publisher _pub_vis_leg_force[4];
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster world_odom_broadcaster;
};
