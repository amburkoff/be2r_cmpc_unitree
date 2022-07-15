#pragma once

#include "Controllers/DesiredStateCommand.h"
#include "controllers/CMPC/Gait_contact.h"
#include "cppTypes.h"
#include <ControlFSMData.h>
#include <FootSwingTrajectory.h>
#include <SparseCMPC/SparseCMPC.h>
#include <Utilities/SpiralIterator.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cstdio>

#define MAX_STEP_HEIGHT 0.18

using Eigen::Array4f;
using Eigen::Array4i;

class VisionMPCLocomotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VisionMPCLocomotion(float _dt, int _iterations_between_mpc, be2r_cmpc_unitree::ros_dynamic_paramsConfig* parameters);
  void initialize();

  void run(ControlFSMData<float>& data, const Vec3<float>& vel_cmd, const grid_map::GridMap& height_map,
           const grid_map::GridMap& height_map_raw);
  bool currently_jumping = false;

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;

private:
  void _SetupCommand(ControlFSMData<float>& data);
  float _updateTrajHeight(size_t foot);
  void _updateFoothold(Vec3<float>& foot, const Vec3<float>& body_pos, const grid_map::GridMap& height_map,
                       const grid_map::GridMap& height_map_raw, int leg);

  void _IdxMapChecking(Vec3<float>& Pf, int x_idx, int y_idx, int& x_idx_selected, int& y_idx_selected,
                       const grid_map::GridMap& height_map, const grid_map::GridMap& height_map_raw, int leg);

  ControlFSMData<float>* _data;

  float _yaw_turn_rate;
  float _yaw_des = 0;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  // High speed running
  float _body_height = 0.29;

  float _body_height_running = 0.29;
  float _body_height_jumping = 0.36;

  void recompute_timing(int iterations_per_mpc);
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode);
  void solveDenseMPC(int* mpcTable, ControlFSMData<float>& data);
  void solveSparseMPC(int* mpcTable, ControlFSMData<float>& data);
  void initSparseMPC();
  int iterationsBetweenMPC;
  be2r_cmpc_unitree::ros_dynamic_paramsConfig* _parameters = nullptr;
  int _gait_period;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGaitContact trotting, trot_contact, standing, walking, two_leg_balance;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  float trajAll[12 * 36];
  ros::Publisher _pub_des_traj[4];
  ros::NodeHandle _nh;
  visualization_msgs::Marker marker[4];
  ros::Publisher _vis_pub[4];

  vectorAligned<Vec12<double>> _sparseTrajectory;

  SparseCMPC _sparseCMPC;
};
