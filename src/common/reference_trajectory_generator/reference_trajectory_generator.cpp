#include "common/reference_trajectory_generator/reference_trajectory_generator.h"

RTG::RTG(LegControllerData<float>* leg_controller_data, StateEstimate<float>* state_estimate)
{
  _rt.x_des.clear();

  _leg_controller_data = leg_controller_data;
  _state_estimate = state_estimate;
}

void RTG::setHorizon(uint16_t horizon)
{
  _horizon = horizon;
}

void RTG::generateTrajectory()
{
}

void RTG::convertTrajectoryToRosPath()
{
  float dt = 0.1;

  _rt.x_des_ros.poses.clear();
  _rt.x_des_ros.header.frame_id = "world";
  _rt.x_des_ros.header.stamp = ros::Time::now();

  // Eigen::Vector3f p_hat(0, 0, 0);
  // Eigen::Vector3f rpy_hat(0, 0, 0);

  // for (size_t i = 0; i < _horizon; i++)
  // {
  //   geometry_msgs::PoseStamped pose_of_traj;

  //   rpy_hat(2) += _cmd.omega_des(2) * dt;

  //   tf::Quaternion quat(tf::Vector3(0, 0, 1), rpy_hat(2));
  //   tf::Matrix3x3 Rot(quat);
  //   Eigen::Matrix<float, 3, 3> R;

  //   for (uint8_t i = 0; i < 3; i++)
  //   {
  //     for (uint8_t j = 0; j < 3; j++)
  //     {
  //       R(i, j) = Rot[i][j];
  //     }
  //   }

  //   p_hat = p_hat + R * _cmd.v_des * dt;
  //   pose_of_traj.pose.position = Eigen::vector3fToPoint(p_hat);
  //   pose_of_traj.pose.orientation.w = _data.quat(0);
  //   pose_of_traj.pose.orientation.x = _data.quat(1);
  //   pose_of_traj.pose.orientation.y = _data.quat(2);
  //   pose_of_traj.pose.orientation.z = _data.quat(3);

  //   _referenceTrajectory.poses.push_back(pose_of_traj);
  // }

  // _pub_ref_traj.publish(_referenceTrajectory);
}

RTG::~RTG()
{
}