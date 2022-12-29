#include "debug.hpp"

Debug::Debug(ros::Time time_start)
  : _zero_time(0),
    _time_start(time_start),
    _tf_buffer(),
    _tf_listener(_tf_buffer)
{
  z_offset = 0.f;
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

  _pub_vis_last_p_stance = _nh.advertise<visualization_msgs::Marker>("/visual/last_p_stance", 1);
  _pub_vis_swing_pf = _nh.advertise<visualization_msgs::Marker>("/visual/swing_pf", 1);
  _pub_vis_estimated_stance_plane = _nh.advertise<visualization_msgs::Marker>("/visual/estimated_stance_plane", 1);
  _pub_vis_leg_des_traj[0] = _nh.advertise<nav_msgs::Path>("/visual/leg0/des_traj", 1);
  _pub_vis_leg_des_traj[1] = _nh.advertise<nav_msgs::Path>("/visual/leg1/des_traj", 1);
  _pub_vis_leg_des_traj[2] = _nh.advertise<nav_msgs::Path>("/visual/leg2/des_traj", 1);
  _pub_vis_leg_des_traj[3] = _nh.advertise<nav_msgs::Path>("/visual/leg3/des_traj", 1);
  _pub_vis_leg_force[0] = _nh.advertise<visualization_msgs::Marker>("/visual/leg0/force", 1);
  _pub_vis_leg_force[1] = _nh.advertise<visualization_msgs::Marker>("/visual/leg1/force", 1);
  _pub_vis_leg_force[2] = _nh.advertise<visualization_msgs::Marker>("/visual/leg2/force", 1);
  _pub_vis_leg_force[3] = _nh.advertise<visualization_msgs::Marker>("/visual/leg3/force", 1);

  _pub_metric_info = _nh.advertise<unitree_legged_msgs::MetricInfo>("/metric_info", 1);
#ifdef PUB_IMU_AND_ODOM
  _pub_odom = _nh.advertise<nav_msgs::Odometry>("/odom", 1);
  _pub_imu = _nh.advertise<sensor_msgs::Imu>("/imu", 1);
#endif
}

void Debug::updateMetrics()
{
  unitree_legged_msgs::MetricInfo data_metric;
  data_metric.final_leg_cost.w = metric_data.final_leg_cost.operator()(0);
  data_metric.final_leg_cost.x = metric_data.final_leg_cost.operator()(1);
  data_metric.final_leg_cost.y = metric_data.final_leg_cost.operator()(2);
  data_metric.final_leg_cost.z = metric_data.final_leg_cost.operator()(3);
  
  data_metric.final_body_cost.w = metric_data.final_body_cost.operator()(0);
  data_metric.final_body_cost.x = metric_data.final_body_cost.operator()(1);
  data_metric.final_body_cost.y = metric_data.final_body_cost.operator()(2);
  data_metric.final_body_cost.z = metric_data.final_body_cost.operator()(3);
  
  data_metric.final_cost.w = metric_data.final_cost.operator()(0);
  data_metric.final_cost.x = metric_data.final_cost.operator()(1);
  data_metric.final_cost.y = metric_data.final_cost.operator()(2);
  data_metric.final_cost.z = metric_data.final_cost.operator()(3);
  _pub_metric_info.publish(data_metric);

}
void Debug::updatePlot()
{
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
    all_legs_info.leg.at(leg_num).mpc_force.x = leg_force[leg_num].x;
    all_legs_info.leg.at(leg_num).mpc_force.y = leg_force[leg_num].y;
    all_legs_info.leg.at(leg_num).mpc_force.z = leg_force[leg_num].z;

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

  odom.header.stamp = time_stamp_udp_get;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";

  odom.pose.pose.position = body_info.pos_act;
  odom.pose.pose.position.z += z_offset;

  geometry_msgs::Quaternion odom_quat;
  odom_quat.x = body_info.quat_act.y;
  odom_quat.y = body_info.quat_act.z;
  odom_quat.z = body_info.quat_act.w;
  odom_quat.w = body_info.quat_act.x;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear = body_info.vel_act.linear;
  odom.twist.twist.angular = body_info.vel_act.angular;

  imu.header.frame_id = "imu_link";
  imu.header.stamp = time_stamp_udp_get;

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

  _drawLastStancePoints();
  _drawSwingFinalPoints();
  _drawEstimatedStancePLane();
  _drawLegsDesiredTrajectory();
  _drawLegsForce();
}

void Debug::tfOdomPublish(ros::Time stamp)
{
  geometry_msgs::TransformStamped odom_trans;

  odom_trans.header.stamp = stamp;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base";

  odom_trans.transform.translation.x = body_info.pos_act.x;
  odom_trans.transform.translation.y = body_info.pos_act.y;
  // odom_trans.transform.translation.z = ground_truth_odom.pose.pose.position.z;
  odom_trans.transform.translation.z = body_info.pos_act.z + z_offset;

  geometry_msgs::Quaternion odom_quat;
  // TODO почему результаты естиматора приходится менять местами?
  odom_quat.x = body_info.quat_act.y;
  odom_quat.y = body_info.quat_act.z;
  odom_quat.z = body_info.quat_act.w;
  odom_quat.w = body_info.quat_act.x;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster.sendTransform(odom_trans);
}

void Debug::tfPublish()
{
  // bool use_map = false;
  // geometry_msgs::TransformStamped odom_corr_transform;
  // if (use_map)
  // {
  //   try
  //   {
  //     odom_corr_transform = _tf_buffer.lookupTransform("corrected_odom", "odom", ros::Time(0));
  //   }
  //   catch (tf2::TransformException& ex)
  //   {
  //     ROS_WARN("%s", ex.what());
  //   }
  // }

  // geometry_msgs::TransformStamped odom_trans_world;

  // odom_trans_world.header.stamp = time_stamp_udp_get;
  // odom_trans_world.header.frame_id = "world";
  // odom_trans_world.child_frame_id = "odom";

  // // z_offset = ground_truth_odom.pose.pose.position.z - body_info.pos_act.z;
  // odom_trans_world.transform.translation.z = 0;
  // odom_trans_world.transform.rotation.w = 1.;

  // world_odom_broadcaster.sendTransform(odom_trans_world);
}

Vec3<float> Debug::_getHipLocation(uint8_t leg_num)
{
  Vec3<float> result(0, 0, 0);

  float x = 0.1805;
  float y = 0.047;

  switch (leg_num)
  {
      // FR
    case 0:
      result(0) = x;
      result(1) = -y;
      break;

      // FL
    case 1:
      result(0) = x;
      result(1) = y;
      break;

      // BR
    case 2:
      result(0) = -x;
      result(1) = -y;
      break;

      // BL
    case 3:
      result(0) = -x;
      result(1) = y;
      break;
  }

  return result;
}

void Debug::_drawLastStancePoints()
{
  visualization_msgs::Marker marker;

  // std::string name = "odom";
  std::string name = "base";

  marker.header.frame_id = name;
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  // pose and orientation must be zero, except orientation.w = 1
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;

  geometry_msgs::Point p0, p1, p2, p3;
  marker.points.clear();

  // p0 = last_p_stance[0]; // FL last stance point
  // p1 = last_p_stance[1]; // FR last stance point
  // p2 = last_p_stance[2]; // BR last stance point
  // p3 = last_p_stance[3]; // BL last stance point
  p0 = last_p_local_stance[0]; // FL last stance local point
  p1 = last_p_local_stance[1]; // FR last stance local point
  p2 = last_p_local_stance[2]; // BR last stance local point
  p3 = last_p_local_stance[3]; // BL last stance local point
  marker.points.push_back(p0);
  marker.points.push_back(p1);
  marker.points.push_back(p3);
  marker.points.push_back(p2);

  _pub_vis_last_p_stance.publish(marker);
}

void Debug::_drawSwingFinalPoints()
{
  visualization_msgs::Marker marker;

  std::string name = "odom";

  marker.header.frame_id = name;
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  // pose and orientation must be zero, except orientation.w = 1
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.025;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  geometry_msgs::Point pf[4];
  marker.points.clear();

  for (size_t i = 0; i < 4; i++)
  {
    pf[i] = all_legs_info.leg[i].swing_pf;
    marker.points.push_back(pf[i]);
  }

  _pub_vis_swing_pf.publish(marker);
}

void Debug::_drawEstimatedStancePLane()
{
  visualization_msgs::Marker marker;

  std::string name = "base";

  float A = mnk_plane.x;
  float B = mnk_plane.y;
  float C = mnk_plane.z;

  float del = sqrt(A * A + B * B + C * C);
  // float roll = -acos(B / del) + M_PI / 2.0;
  // float pitch = acos(A / del) - M_PI / 2.0;
  float roll = -acos(B / del) + M_PI / 2.0;
  float pitch = acos(A / del) - M_PI / 2.0;

  // body_info.pos_z_global = abs(De) / sqrt(A * A + B * B + C * C);

  tf::Quaternion quat;
  quat.setRPY(roll, pitch, 0.0);
  quat.normalize();

  marker.header.frame_id = name;
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  // pose and orientation must be zero, except orientation.w = 1
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 1.0 / C;
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.00001;
  marker.color.a = 0.7;
  marker.color.r = 0.5;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  _pub_vis_estimated_stance_plane.publish(marker);
}

void Debug::_drawLegsDesiredTrajectory()
{
  for (size_t i = 0; i < 4; i++)
  {
    leg_traj_des[i].header.frame_id = "odom";
    _pub_vis_leg_des_traj[i].publish(leg_traj_des[i]);
  }
}

void Debug::_drawLegsForce()
{
  visualization_msgs::Marker marker;
  std::string names[4] = { "FR_hip", "FL_hip", "RR_hip", "RL_hip" };
  float koef = 500;

  for (size_t i = 0; i < 4; i++)
  {
    marker.header.frame_id = names[i];
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    // pose and orientation must be zero, except orientation.w = 1
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.005; // shaft diameter
    marker.scale.y = 0.01;  // head diameter
    marker.scale.z = 0.0;   // if not zero, specifies head length
    marker.color.a = 0.8;   // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    geometry_msgs::Point p1, p2;
    // start point
    p1 = all_legs_info.leg[i].p_act;
    // finish point
    p2.x = all_legs_info.leg[i].p_act.x + (-leg_force[i].x / koef);
    p2.y = all_legs_info.leg[i].p_act.y + (-leg_force[i].y / koef);
    p2.z = all_legs_info.leg[i].p_act.z + (-leg_force[i].z / koef);
    marker.points.clear();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    _pub_vis_leg_force[i].publish(marker);
  }
}
