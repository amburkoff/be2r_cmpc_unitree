#pragma once

#include "FloatingBaseModel.h"
#include "Gait_contact.h"
#include "cppTypes.h"
#include <ControlFSMData.h>
#include <FootSwingTrajectory.h>
#include <SparseCMPC/SparseCMPC.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cstdio>

using Eigen::Array4f;
using Eigen::Array4i;

class CMPCLocomotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CMPCLocomotion(float _dt, int _iterations_between_mpc, ControlFSMData<float>* data);
  void initialize();

  void run(ControlFSMData<float>& data);
  void myNewVersion(ControlFSMData<float>& data);
  void myLQRVersion(ControlFSMData<float>& data);
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

  float _yaw_turn_rate;
  float _yaw_des = 0;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  float _body_height = 0.29;
  float _swing_trajectory_height = 0.09;

  float _body_height_running = 0.29;
  float _body_height_jumping = 0.36;

  FloatingBaseModel<float> _model;
  FBModelState<float> _state;
  DMat<float> _A;
  DMat<float> _Ainv;
  DVec<float> _grav;
  DVec<float> _coriolis;
  ControlFSMData<float>* _data;

  //----------------------- LQR -------------------------
  Eigen::Matrix<double, 3, 3> cross_mat(Eigen::Matrix<double, 3, 3> I_inv, Eigen::Matrix<double, 3, 1> r);
  Eigen::Matrix<double, 12, 1> calcLinError();
  void updateALQR(uint16_t contacts);
  void updateBLQR(uint16_t contacts);
  void inverseCrossMatrix(const Eigen::MatrixXd& R, Eigen::VectorXd& omega);
  void crossMatrix(Eigen::MatrixXd& R, const Eigen::VectorXd& omega);
  void matrixLogRot(const Eigen::MatrixXd& R, Eigen::VectorXd& omega);

  uint8_t leg_contact_num[4] = { 0, 0, 0, 0 };

  Eigen::Matrix<double, 12, 12> A;
  Eigen::MatrixXd B;
  Eigen::VectorXd f_ref_world;
  Eigen::MatrixXd p_feet_desired; // new

  Eigen::VectorXd _x_COM_world;
  Eigen::VectorXd _x_COM_world_desired;
  Eigen::VectorXd _xdot_COM_world;
  Eigen::VectorXd _xdot_COM_world_desired;
  Eigen::Matrix<double, 3, 3> _R_b_world;
  Eigen::Matrix<double, 3, 3> _R_b_world_desired;
  Eigen::VectorXd _omega_b_body;
  Eigen::VectorXd _omega_b_body_desired;

  Eigen::VectorXd _error_x_lin;
  Eigen::VectorXd _error_dx_lin;
  Eigen::VectorXd _error_R_lin;
  Eigen::VectorXd _error_omega_lin;
  //----------------------- LQR -------------------------

  void recompute_timing(int iterations_per_mpc);
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode);
  void solveDenseMPC(int* mpcTable, ControlFSMData<float>& data);
  void _updateModel(const StateEstimate<float>& state_est, const LegControllerData<float>* leg_data);
  int iterationsBetweenMPC;
  be2r_cmpc_unitree::ros_dynamic_paramsConfig* _parameters = nullptr;
  int _gait_period;
  int _gait_period_long;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGaitContact trotting, trot_contact, standing, walking, two_leg_balance, give_hand, trot_long;
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
  float pitch_cmd = 0;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  float trajAll[12 * 36];
  ros::NodeHandle _nh;

  vectorAligned<Vec12<double>> _sparseTrajectory;

  SparseCMPC _sparseCMPC;
};
