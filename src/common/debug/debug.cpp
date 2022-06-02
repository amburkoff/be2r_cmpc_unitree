#include "debug.hpp"

Debug::Debug(ros::Time time_start) : _zero_time(0), _time_start(time_start)
{
  _init();
}

void Debug::_init()
{
  _initPublishers();

  body_info.quat_act.w = 1.0;
}

void Debug::_initPublishers()
{
  _pub_joint_states = _nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
  _pub_all_legs_info = _nh.advertise<unitree_legged_msgs::AllLegsInfo>("/all_legs_info", 1);
  _pub_body_info = _nh.advertise<unitree_legged_msgs::BodyInfo>("/body_info", 1);

#ifdef PUB_IMU_AND_ODOM
  _pub_odom = _nh.advertise<nav_msgs::Odometry>("/odom", 1);
  _pub_imu = _nh.advertise<sensor_msgs::Imu>("/imu", 1);
#endif
}

void Debug::updatePlot()
{
  ros::Duration delta_t = ros::Time::now() - _time_start;

  // all_legs_info.header.stamp = _zero_time + delta_t;
  // body_info.header.stamp = _zero_time + delta_t;
  all_legs_info.header.stamp = ros::Time::now();
  body_info.header.stamp = ros::Time::now();

  for (size_t leg_num = 0; leg_num < 4; leg_num++)
  {
    all_legs_info.leg.at(leg_num).p_error.x = all_legs_info.leg.at(leg_num).p_des.x - all_legs_info.leg.at(leg_num).p_act.x;
    all_legs_info.leg.at(leg_num).p_error.y = all_legs_info.leg.at(leg_num).p_des.y - all_legs_info.leg.at(leg_num).p_act.y;
    all_legs_info.leg.at(leg_num).p_error.z = all_legs_info.leg.at(leg_num).p_des.z - all_legs_info.leg.at(leg_num).p_act.z;

    all_legs_info.leg.at(leg_num).v_error.x = all_legs_info.leg.at(leg_num).v_des.x - all_legs_info.leg.at(leg_num).v_act.x;
    all_legs_info.leg.at(leg_num).v_error.y = all_legs_info.leg.at(leg_num).v_des.y - all_legs_info.leg.at(leg_num).v_act.y;
    all_legs_info.leg.at(leg_num).v_error.z = all_legs_info.leg.at(leg_num).v_des.z - all_legs_info.leg.at(leg_num).v_act.z;
  }

  body_info.state_error.p.x = body_info.pos_des.x - body_info.pos_act.x;
  body_info.state_error.p.y = body_info.pos_des.y - body_info.pos_act.y;
  body_info.state_error.p.z = body_info.pos_des.z - body_info.pos_act.z;

  body_info.state_error.euler.x = body_info.euler_des.x - body_info.euler_act.x;
  body_info.state_error.euler.y = body_info.euler_des.y - body_info.euler_act.y;
  body_info.state_error.euler.z = body_info.euler_des.z - body_info.euler_act.z;

  body_info.state_error.v.linear.x = body_info.vel_des.linear.x - body_info.vel_act.linear.x;
  body_info.state_error.v.linear.y = body_info.vel_des.linear.y - body_info.vel_act.linear.y;
  body_info.state_error.v.linear.z = body_info.vel_des.linear.z - body_info.vel_act.linear.z;

  body_info.state_error.v.angular.x = body_info.vel_des.angular.x - body_info.vel_act.angular.x;
  body_info.state_error.v.angular.y = body_info.vel_des.angular.y - body_info.vel_act.angular.y;
  body_info.state_error.v.angular.z = body_info.vel_des.angular.z - body_info.vel_act.angular.z;

  _pub_all_legs_info.publish(all_legs_info);
  _pub_body_info.publish(body_info);

#ifdef PUB_IMU_AND_ODOM
  nav_msgs::Odometry odom;

  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";

  odom.pose.pose.position = body_info.pos_act;

  geometry_msgs::Quaternion odom_quat;
  odom_quat.x = body_info.quat_act.y;
  odom_quat.y = body_info.quat_act.z;
  odom_quat.z = body_info.quat_act.w;
  odom_quat.w = body_info.quat_act.x;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear = body_info.vel_act.linear;
  odom.twist.twist.angular = body_info.vel_act.angular;

  imu.header.frame_id = "imu_link";
  imu.header.stamp = ros::Time::now();

  imu.orientation = odom_quat;

  _pub_odom.publish(odom);
  _pub_imu.publish(imu);
#endif
}

void Debug::updateVisualization()
{
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.name.push_back("FR_hip_joint");
  msg.name.push_back("FR_thigh_joint");
  msg.name.push_back("FR_calf_joint");

  msg.name.push_back("FL_hip_joint");
  msg.name.push_back("FL_thigh_joint");
  msg.name.push_back("FL_calf_joint");

  msg.name.push_back("RR_hip_joint");
  msg.name.push_back("RR_thigh_joint");
  msg.name.push_back("RR_calf_joint");

  msg.name.push_back("RL_hip_joint");
  msg.name.push_back("RL_thigh_joint");
  msg.name.push_back("RL_calf_joint");

  for (size_t leg_num = 0; leg_num < 4; leg_num++)
  {
    for (uint8_t joint_num = 0; joint_num < 3; joint_num++)
    {
      msg.position.push_back(all_legs_info.leg.at(leg_num).joint.at(joint_num).q);
      msg.velocity.push_back(all_legs_info.leg.at(leg_num).joint.at(joint_num).dq);
    }
  }

  _pub_joint_states.publish(msg);
}

void Debug::tfPublish()
{
  geometry_msgs::TransformStamped odom_trans;

  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base";

  odom_trans.transform.translation.x = body_info.pos_act.x;
  odom_trans.transform.translation.y = body_info.pos_act.y;
  odom_trans.transform.translation.z = body_info.pos_act.z;

  geometry_msgs::Quaternion odom_quat;
  // TODO почему результаты естиматора приходится менять местами?
  odom_quat.x = body_info.quat_act.y;
  odom_quat.y = body_info.quat_act.z;
  odom_quat.z = body_info.quat_act.w;
  odom_quat.w = body_info.quat_act.x;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster.sendTransform(odom_trans);

  geometry_msgs::TransformStamped ground_truth_tf;

  ground_truth_tf.header.stamp = odom_trans.header.stamp;
  ground_truth_tf.header.frame_id = "world";
  ground_truth_tf.child_frame_id = "odom";

  ground_truth_tf.transform.translation.x = 0;
  ground_truth_tf.transform.translation.y = 0;
  ground_truth_tf.transform.translation.z = ground_truth_odom.pose.pose.position.z - body_info.pos_act.z;

  ground_truth_tf.transform.rotation.x = 0;
  ground_truth_tf.transform.rotation.y = 0;
  ground_truth_tf.transform.rotation.z = 0;
  ground_truth_tf.transform.rotation.w = 1;

  world_broadcaster.sendTransform(ground_truth_tf);
}
