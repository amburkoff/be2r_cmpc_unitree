#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>
#include <cmath>
#include <iostream>
#include <iterator>
#include <math.h>
#include <streambuf>

#include "CMPC_Locomotion.h"
#include "ControlFSMData.h"
#include "GraphSearch.h"
#include "convexMPC_interface.h"

#include "Gait.h"

#define GAIT_PERIOD_WALKING 32

// #define SHOW_MPC_SOLVE_TIME

using namespace std;

////////////////////
// Controller
////////////////////

CMPCLocomotion::CMPCLocomotion(float _dt, int _iterations_between_mpc, ControlFSMData<float>* data)
  : _data(data),
    iterationsBetweenMPC(_iterations_between_mpc),
    _parameters(_data->userParameters),
    _gait_period(_parameters->gait_period),
    _gait_period_long(32),
    horizonLength(_data->staticParams->horizon),
    dt(_dt),
    trotting(_gait_period, Vec4<int>(0, _gait_period / 2.0, _gait_period / 2.0, 0), Vec4<int>(_gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0), "Trotting"),
    trot_long(_gait_period_long, Vec4<int>(0, _gait_period_long / 2.0, _gait_period_long / 2.0, 0), Vec4<int>(24, 24, 24, 24), "Trotting long"),
    trot_contact(_gait_period, Vec4<int>(0, _gait_period / 2.0, _gait_period / 2.0, 0), Vec4<int>(_gait_period * 0.25, _gait_period * 0.25, _gait_period * 0.25, _gait_period * 0.25), "Trot contact"),
    standing(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(_gait_period, _gait_period, _gait_period, _gait_period), "Standing"),
    give_hand(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(_gait_period, _gait_period, _gait_period, _gait_period), "GiveHand"),
    walking(GAIT_PERIOD_WALKING, Vec4<int>(2 * GAIT_PERIOD_WALKING / 4., 0, GAIT_PERIOD_WALKING / 4., 3 * GAIT_PERIOD_WALKING / 4.), Vec4<int>(0.75 * GAIT_PERIOD_WALKING, 0.75 * GAIT_PERIOD_WALKING, 0.75 * GAIT_PERIOD_WALKING, 0.75 * GAIT_PERIOD_WALKING),
            "Walking"), // for real
    two_leg_balance(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(_gait_period, _gait_period, _gait_period, 0), "Two legs balance")
{
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[CMPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  // setup_problem(dtMPC, horizonLength, 0.4, 150); // original
  setup_problem(dtMPC, horizonLength, 0.4, 300); // original

  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();

  _model = _data->quadruped->buildModel();
  _state.q = DVec<float>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<float>::Zero(cheetah::num_act_joint);
}

void CMPCLocomotion::initialize()
{
  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  firstRun = true;

  iterationCounter = 0;
}

void CMPCLocomotion::recompute_timing(int iterations_per_mpc)
{
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void CMPCLocomotion::_SetupCommand(ControlFSMData<float>& data)
{
  float x_vel_cmd, y_vel_cmd;
  float filter_x(0.01);
  float filter_y(0.01);

  _yaw_turn_rate = data.gamepad_command->right_stick_analog[0];
  x_vel_cmd = data.gamepad_command->left_stick_analog[1];
  y_vel_cmd = data.gamepad_command->left_stick_analog[0];

  _yaw_turn_rate *= data.staticParams->max_turn_rate;
  x_vel_cmd *= data.staticParams->max_vel_x;
  y_vel_cmd *= data.staticParams->max_vel_y;

  pitch_cmd = 0.4 * data.gamepad_command->right_stick_analog[1];

  _x_vel_des = _x_vel_des * (1 - filter_x) + x_vel_cmd * filter_x;
  _y_vel_des = _y_vel_des * (1 - filter_y) + y_vel_cmd * filter_y;

  if ((M_PI - abs(_yaw_des)) <= 0.1)
  {
    _yaw_des = data.stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  }
  else
  {
    _yaw_des += dt * _yaw_turn_rate;
  }

  if (current_gait == 13 || current_gait == 4 || current_gait == 11)
  {
    world_position_desired[0] = stand_traj[0] + 0.05 * data.gamepad_command->left_stick_analog[1];
    world_position_desired[1] = stand_traj[1] + 0.05 * data.gamepad_command->left_stick_analog[0];
    _body_height = stand_traj[2];

    _pitch_des = 0.2 * data.gamepad_command->right_stick_analog[1];
    _yaw_des = stand_traj[5] + 0.2 * data.gamepad_command->right_stick_analog[0];
  }

  _body_height = _parameters->body_height;
  _swing_trajectory_height = _parameters->Swing_traj_height;

  if (data.gamepad_command->triangle && (gaitNumber == 15))
  {
    data.gamepad_command->stairs_mode = StairsMode::UP;
    _body_height = 0.28;
    _swing_trajectory_height = 0.17;
  }

  if (data.gamepad_command->cross && (gaitNumber == 15))
  {
    data.gamepad_command->stairs_mode = StairsMode::DOWN;
    _body_height = 0.25;
    _swing_trajectory_height = 0.06;
  }

  // Update PD coefs
  Kp = Vec3<float>(_parameters->Kp_cartesian_0, _parameters->Kp_cartesian_1, _parameters->Kp_cartesian_2).asDiagonal();
  Kp_stance = Kp;

  Kd = Vec3<float>(_parameters->Kd_cartesian_0, _parameters->Kd_cartesian_1, _parameters->Kd_cartesian_2).asDiagonal();
  Kd_stance = Kd;
}

void CMPCLocomotion::run(ControlFSMData<float>& data)
{
  // myNewVersion(data);
  myLQRVersion(data);
}

void CMPCLocomotion::myNewVersion(ControlFSMData<float>& data)
{
  bool omniMode = false;

  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;

  auto& seResult = data.stateEstimator->getResult();

  // Check if transition to standing
  if (((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = seResult.position[2];
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait_contact* gait = &trotting;
  current_gait = gaitNumber;

  if (current_gait == 9)
  {
    gait = &trotting;
  }
  else if (current_gait == 15)
  {
    gait = &trot_long;
  }
  else if (current_gait == 4)
  {
    gait = &standing;
  }

  // gait->updatePeriod(_dyn_params->gait_period);
  gait->restoreDefaults();
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  gait->earlyContactHandle(seResult.contactSensor, iterationsBetweenMPC, iterationCounter);

  recompute_timing(default_iterations_between_mpc);

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  // Integral-esque pitche and roll compensation
  if (fabs(v_robot[0]) > .2) // avoid dividing by zero
  {
    rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
  }
  if (fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber != 8); // turn off for pronking

  static float z_des[4] = { 0 };

  if (current_gait == 4 || current_gait == 13)
  {
    _pitch_des = 0.0;
  }
  else if (current_gait != 11)
  {
    // estimated pitch of plane and pitch correction depends on Vdes
    // _pitch_des = pitch_cmd + data.stateEstimator->getResult().rpy[1] + data.stateEstimator->getResult().est_pitch_plane + 0.1;
    _pitch_des = pitch_cmd + data.stateEstimator->getResult().rpy[1] + data.stateEstimator->getResult().est_pitch_plane;

    if (_x_vel_des > 0)
    {
      _pitch_des += -0.3 * _x_vel_des / data.staticParams->max_vel_x;
    }
    else
    {
      _pitch_des += -0.2 * _x_vel_des / data.staticParams->max_vel_x;
    }
  }

  for (int i = 0; i < 4; i++)
  {
    pFoot[i] = seResult.position + seResult.rBody.transpose() * (data.quadruped->getHipLocation(i) + data.legController->datas[i].p);
  }

  if (gait != &standing)
  {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if (firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];
    _yaw_des = seResult.rpy[2];

    for (int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(_swing_trajectory_height);

      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_ps.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_ps.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_ps.z = pFoot[i](2);

      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_pf.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_pf.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_pf.z = pFoot[i](2);

      z_des[i] = pFoot[i](2);
    }

    firstRun = false;
  }

  // foot placement
  for (int l = 0; l < 4; l++)
  {
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);
  }

  float side_sign[4] = { -1, 1, -1, 1 };
  float interleave_y[4] = { -0.08, 0.08, 0.02, -0.02 };
  float interleave_gain = -0.2;
  float v_abs = std::fabs(v_des_robot[0]);

  // calculate swing pf
  for (int i = 0; i < 4; i++)
  {
    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }

    footSwingTrajectories[i].setHeight(_swing_trajectory_height);

    Vec3<float> offset(0, side_sign[i] * data.quadruped->_abadLinkLength, 0);

    Vec3<float> pRobotFrame = (data.quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);

    Vec3<float> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

    float p_rel_max = 0.3f;

    float pfx_rel = seResult.vWorld[0] * stance_time / 2.0 + 0.03f * (v_des_world[0] - seResult.vWorld[0]) + 0.5f * seResult.position[2] / 9.81f * seResult.vWorld[1] * _yaw_turn_rate;
    float pfy_rel = seResult.vWorld[1] * stance_time / 2.0 + 0.03f * (v_des_world[1] - seResult.vWorld[1]) - 0.5f * seResult.position[2] / 9.81f * seResult.vWorld[0] * _yaw_turn_rate;

    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    Pf[2] = z_des[i];

    footSwingTrajectories[i].setFinalPosition(Pf);
    data.debug->all_legs_info.leg[i].swing_pf.x = Pf(0);
    data.debug->all_legs_info.leg[i].swing_pf.y = Pf(1);
    data.debug->all_legs_info.leg[i].swing_pf.z = Pf(2);
  }

  // calc gait
  iterationCounter++;

  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();

  // updateMPCIfNeeded(mpcTable, data, omniMode);

  Vec4<float> se_contactState(0, 0, 0, 0);
  static bool is_stance[4] = { 0, 0, 0, 0 };
  static Vec3<float> p_fw[4] = {};
  static Vec3<float> p_fl[4] = {};
  static float delta_yaw[4] = {};
  static Vec3<float> delta_p_bw[4] = {};

  for (int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];

    if ((is_stance[foot] == 0) && !(swingState > 0))
    {
      is_stance[foot] = 1;

      data.debug->last_p_stance[foot] = ros::toMsg(pFoot[foot]);
      p_fw[foot] = pFoot[foot];

      p_fl[foot] = data.legController->datas[foot].p + data.quadruped->getHipLocation(foot);
      delta_p_bw[foot] << 0, 0, 0;
      delta_yaw[foot] = 0;
    }

    delta_p_bw[foot] += seResult.vBody * dt;
    delta_yaw[foot] += seResult.omegaBody(2) * dt;
    data.debug->last_p_local_stance[foot] = ros::toMsg(ori::rpyToRotMat(Vec3<float>(0, 0, delta_yaw[foot])) * (p_fl[foot] - delta_p_bw[foot]));

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        is_stance[foot] = 0;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);

        z_des[foot] = pFoot[foot][2];
      }

      // for visual -----------------------------------------------------------------------
      geometry_msgs::PoseStamped pose_traj;
      Vec3<float> p_des_traj(0, 0, 0);
      data.debug->leg_traj_des[foot].poses.clear();
      data.debug->leg_traj_des[foot].header.stamp = ros::Time::now();

      // 20 segment trajcetory
      for (size_t i = 0; i < 21; i++)
      {
        footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + ((1.0 - swingState) / 21.0 * (float)i), swingTimes[foot]);
        p_des_traj = footSwingTrajectories[foot].getPosition();

        pose_traj.pose.position = ros::toMsg(p_des_traj);

        data.debug->leg_traj_des[foot].poses.push_back(pose_traj);
      }
      // for visual -----------------------------------------------------------------------

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data.quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      Vec3<float> vActFootWorld = seResult.rBody.inverse() * (data.legController->datas[foot].v) + seResult.vWorld;

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

      data.debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesLeg);
      data.debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesLeg);
      data.debug->all_legs_info.leg.at(foot).p_w_act = ros::toMsg(pFoot[foot]);
      data.debug->all_legs_info.leg.at(foot).v_w_act = ros::toMsg(vActFootWorld);
      data.debug->all_legs_info.leg.at(foot).p_w_des = ros::toMsg(pDesFootWorld);
      data.debug->all_legs_info.leg.at(foot).v_w_des = ros::toMsg(vDesFootWorld);

      Eigen::Vector3f f = Eigen::Vector3f(0, 0, 0.0);
      Fr_des[foot] = -seResult.rBody * f;
      f_ff[foot] = Fr_des[foot];

      data.debug->leg_force[foot] = ros::toMsg(f_ff[foot]);

      if (!data.userParameters->use_wbc)
      {
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = Kp;
        data.legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

      // for visual ---------------------------------------------------------
      data.debug->leg_traj_des[foot].poses.clear();
      data.debug->leg_traj_des[foot].header.stamp = ros::Time::now();
      data.debug->all_legs_info.leg[foot].swing_pf = ros::toMsg(pFoot[foot]);
      // for visual ---------------------------------------------------------

      geometry_msgs::Point point;
      point = ros::toMsg(data.legController->datas[foot].p + data.quadruped->getHipLocation(foot));
      data.debug->last_p_local_stance[foot] = point;

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      // Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> vDesFootWorld = Vec3<float>::Zero();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data.quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      Vec3<float> vActFootWorld = seResult.rBody.inverse() * (data.legController->datas[foot].v) + seResult.vWorld;

      // temporary debug
      data.debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesLeg);
      data.debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesLeg);

      if (!data.userParameters->use_wbc) // wbc off
      {
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = Kp_stance;
        data.legController->commands[foot].kdCartesian = Kd_stance;

        data.legController->commands[foot].forceFeedForward = f_ff[foot];
        data.legController->commands[foot].kdJoint = Vec3<float>(_parameters->Kd_joint_0, _parameters->Kd_joint_1, _parameters->Kd_joint_2).asDiagonal();
      }
      else
      { // Stance foot damping
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = 0. * Kp_stance;
        data.legController->commands[foot].kdCartesian = Kd_stance;
      }

      Eigen::Vector3f f = Eigen::Vector3f(0, 0, 6.0 * 9.81);
      f_ff[foot] = -seResult.rBody * f;
      Fr_des[foot] = f;

      se_contactState[foot] = contactState;

      data.debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesLeg);
      data.debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesLeg);
      data.debug->all_legs_info.leg.at(foot).p_w_act = ros::toMsg(pFoot[foot]);
      data.debug->all_legs_info.leg.at(foot).v_w_act = ros::toMsg(vActFootWorld);
      data.debug->all_legs_info.leg.at(foot).p_w_des = ros::toMsg(pDesFootWorld);
      data.debug->all_legs_info.leg.at(foot).v_w_des = ros::toMsg(vDesFootWorld);

      data.debug->leg_force[foot] = ros::toMsg(f_ff[foot]);
    }
  }

  data.stateEstimator->setContactPhase(se_contactState);
  data.stateEstimator->setSwingPhase(gait->getSwingState());

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.0;
  pBody_RPY_des[1] = 0.0;
  // pBody_RPY_des[1] = _pitch_des;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  data.debug->body_info.pos_des.x = pBody_des[0];
  data.debug->body_info.pos_des.y = pBody_des[1];
  data.debug->body_info.pos_des.z = pBody_des[2];

  data.debug->body_info.vel_des.linear.x = vBody_des[0];
  data.debug->body_info.vel_des.linear.y = vBody_des[1];
  data.debug->body_info.vel_des.linear.z = vBody_des[2];

  data.debug->body_info.euler_des.x = pBody_RPY_des[0];
  data.debug->body_info.euler_des.y = pBody_RPY_des[1];
  data.debug->body_info.euler_des.z = pBody_RPY_des[2];

  data.debug->body_info.vel_des.angular.x = vBody_Ori_des[0];
  data.debug->body_info.vel_des.angular.y = vBody_Ori_des[1];
  data.debug->body_info.vel_des.angular.z = vBody_Ori_des[2];

  contact_state = gait->getContactState();
  // END of WBC Update
}

void CMPCLocomotion::updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode)
{
  if ((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data.stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;

    const float max_pos_error = .1;
    float xStart = world_position_desired[0];
    float yStart = world_position_desired[1];

    if (xStart - p[0] > max_pos_error)
    {
      xStart = p[0] + max_pos_error;
    }
    if (p[0] - xStart > max_pos_error)
    {
      xStart = p[0] - max_pos_error;
    }

    if (yStart - p[1] > max_pos_error)
    {
      yStart = p[1] + max_pos_error;
    }
    if (p[1] - yStart > max_pos_error)
    {
      yStart = p[1] - max_pos_error;
    }

    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;

    float trajInitial[12] = { pBody_RPY_des[0], // 0 roll des
                              pBody_RPY_des[1], // 1 pitch des
                              pBody_RPY_des[2], // 2 yaw des
                              pBody_des[0],     // 3 x body des
                              pBody_des[1],     // 4 y body des
                              pBody_des[2],     // 5 z body des
                              vBody_Ori_des[0], // 6 velocity roll des
                              vBody_Ori_des[1], // 7 velocity pitch des
                              vBody_Ori_des[2], // 8 velocity yaw des
                              vBody_des[0],     // 9 vx body des
                              vBody_des[1],     // 10 vy body des
                              vBody_des[2] };   // 11 vz body des

    // i - horizon step, j - traj element
    for (int i = 0; i < horizonLength; i++)
    {
      for (int j = 0; j < 12; j++)
      {
        trajAll[12 * i + j] = trajInitial[j];
      }

      if (i > 0)
      {
        trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
        trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
      }
    }

    // dense matrix - contain mostly NON zero values
    solveDenseMPC(mpcTable, data);
  }
}

void CMPCLocomotion::solveDenseMPC(int* mpcTable, ControlFSMData<float>& data)
{
  auto seResult = data.stateEstimator->getResult();

  // Q roll pitch yaw x_des y_des z_des v_roll_des v_pitch_des v_yaw_des vx_des
  // vy_des vz_des
  //  original
  float Q[12] = { 0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1 };

  float Q_roll = data.staticParams->Q_roll;
  float Q_pitch = data.staticParams->Q_pitch;
  float Q_yaw = data.staticParams->Q_yaw;
  float Q_x = data.staticParams->Q_x;
  float Q_y = data.staticParams->Q_y;
  float Q_z = data.staticParams->Q_z;
  float Q_w_roll = data.staticParams->Q_w_roll;
  float Q_w_pitch = data.staticParams->Q_w_pitch;
  float Q_w_yaw = data.staticParams->Q_w_yaw;
  float Q_vx = data.staticParams->Q_vx;
  float Q_vy = data.staticParams->Q_vy;
  float Q_vz = data.staticParams->Q_vz;

  Q[0] = Q_roll;
  Q[1] = Q_pitch;
  Q[2] = Q_yaw;
  Q[3] = Q_x;
  Q[4] = Q_y;
  Q[5] = Q_z;
  Q[6] = Q_w_roll;
  Q[7] = Q_w_pitch;
  Q[8] = Q_w_yaw;
  Q[9] = Q_vx;
  Q[10] = Q_vy;
  Q[11] = Q_vz;

  float roll = seResult.rpy[0];
  float pitch = seResult.rpy[1];
  float yaw = seResult.rpy[2];
  float* weights = Q;
  // float alpha = 4e-5; // make setting eventually
  float alpha = data.staticParams->alpha;
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for (int i = 0; i < 12; i++)
  {
    r[i] = pFoot[i % 4][i / 4] - seResult.position[i / 4];
  }

  if (alpha > 1e-4)
  {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC, horizonLength, 0.4, 120);
  update_x_drag(x_comp_integral);

  if (vxy[0] > 0.3 || vxy[0] < -0.3)
  {
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho, _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
  // t1.stopPrint("Setup MPC");
  // printf("MPC Setup time %f ms\n", t1.getMs());

  Timer t2;
  // cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p, v, q, w, r, roll, pitch, yaw, weights, trajAll, alpha, mpcTable);
  // t2.stopPrint("Run MPC");
  // printf("MPC Solve time %f ms\n", t2.getMs());

  for (int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;

    for (int axis = 0; axis < 3; axis++)
    {
      f[axis] = get_solution(leg * 3 + axis);
    }

    f_ff[leg] = -seResult.rBody * f;

    // Update for WBC
    Fr_des[leg] = f;
  }
}

void CMPCLocomotion::_updateModel(const StateEstimate<float>& state_est, const LegControllerData<float>* leg_data)
{
  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;

  for (size_t i(0); i < 3; ++i)
  {
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i + 3] = state_est.vBody[i];

    for (size_t leg(0); leg < 4; ++leg)
    {
      _state.q[3 * leg + i] = leg_data[leg].q[i];
      _state.qd[3 * leg + i] = leg_data[leg].qd[i];
    }
  }

  _model.setState(_state);

  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();

  // cout << "Mass Matrix: " << _A << endl;
  // cout << "C+G: " << _grav+_coriolis << endl;
  // cout << "Gravity: " << _grav << endl;
}

void CMPCLocomotion::myLQRVersion(ControlFSMData<float>& data)
{
  bool omniMode = false;

  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;

  auto& seResult = data.stateEstimator->getResult();

  // Check if transition to standing
  if (((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = seResult.position[2];
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait_contact* gait = &trotting;
  current_gait = gaitNumber;

  if (current_gait == 9)
  {
    gait = &trotting;
  }
  else if (current_gait == 15)
  {
    gait = &trot_long;
  }
  else if (current_gait == 4)
  {
    gait = &standing;
  }

  // gait->updatePeriod(_dyn_params->gait_period);
  gait->restoreDefaults();
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  gait->earlyContactHandle(seResult.contactSensor, iterationsBetweenMPC, iterationCounter);

  recompute_timing(default_iterations_between_mpc);

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;
  static float z_des[4] = { 0 };

  if (current_gait == 4 || current_gait == 13)
  {
    _pitch_des = 0.0;
  }
  else if (current_gait != 11)
  {
    // estimated pitch of plane and pitch correction depends on Vdes
    // _pitch_des = pitch_cmd + data.stateEstimator->getResult().rpy[1] + data.stateEstimator->getResult().est_pitch_plane + 0.1;
    _pitch_des = pitch_cmd + data.stateEstimator->getResult().rpy[1] + data.stateEstimator->getResult().est_pitch_plane;

    if (_x_vel_des > 0)
    {
      _pitch_des += -0.3 * _x_vel_des / data.staticParams->max_vel_x;
    }
    else
    {
      _pitch_des += -0.2 * _x_vel_des / data.staticParams->max_vel_x;
    }
  }

  for (int i = 0; i < 4; i++)
  {
    pFoot[i] = seResult.position + seResult.rBody.transpose() * (data.quadruped->getHipLocation(i) + data.legController->datas[i].p);
  }

  if (gait != &standing)
  {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if (firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];
    _yaw_des = seResult.rpy[2];

    for (int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(_swing_trajectory_height);

      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_ps.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_ps.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_ps.z = pFoot[i](2);

      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_pf.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_pf.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_pf.z = pFoot[i](2);

      z_des[i] = pFoot[i](2);
    }

    firstRun = false;
  }

  // foot placement
  for (int l = 0; l < 4; l++)
  {
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);
  }

  float side_sign[4] = { -1, 1, -1, 1 };
  float interleave_y[4] = { -0.08, 0.08, 0.02, -0.02 };
  float interleave_gain = -0.2;
  float v_abs = std::fabs(v_des_robot[0]);

  // calculate swing pf
  for (int i = 0; i < 4; i++)
  {
    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }

    footSwingTrajectories[i].setHeight(_swing_trajectory_height);

    Vec3<float> offset(0, side_sign[i] * data.quadruped->_abadLinkLength, 0);

    Vec3<float> pRobotFrame = (data.quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);

    Vec3<float> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

    float p_rel_max = 0.3f;

    float pfx_rel = seResult.vWorld[0] * stance_time / 2.0 + 0.03f * (v_des_world[0] - seResult.vWorld[0]) + 0.5f * seResult.position[2] / 9.81f * seResult.vWorld[1] * _yaw_turn_rate;
    float pfy_rel = seResult.vWorld[1] * stance_time / 2.0 + 0.03f * (v_des_world[1] - seResult.vWorld[1]) - 0.5f * seResult.position[2] / 9.81f * seResult.vWorld[0] * _yaw_turn_rate;

    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    Pf[2] = z_des[i];

    footSwingTrajectories[i].setFinalPosition(Pf);
    data.debug->all_legs_info.leg[i].swing_pf.x = Pf(0);
    data.debug->all_legs_info.leg[i].swing_pf.y = Pf(1);
    data.debug->all_legs_info.leg[i].swing_pf.z = Pf(2);
  }

  // calc gait
  iterationCounter++;

  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();

  Vec4<float> se_contactState(0, 0, 0, 0);
  static bool is_stance[4] = { 0, 0, 0, 0 };
  static Vec3<float> p_fw[4] = {};
  static Vec3<float> p_fl[4] = {};
  static float delta_yaw[4] = {};
  static Vec3<float> delta_p_bw[4] = {};

  uint8_t contacts = 0;
  uint8_t leg_contact_num[4] = { 0, 0, 0, 0 };
  for (uint8_t i = 0; i < 4; i++)
  {
    if (contactStates[i] > 0)
    {
      leg_contact_num[contacts] = i;
      contacts++;
    }
  }

  // cout << "contacts: " << (int)contacts << endl;
  // cout << "contacts leg: " << (int)leg_contact_num[0] << endl;
  // cout << "contacts leg: " << (int)leg_contact_num[1] << endl;
  // cout << "contacts leg: " << (int)leg_contact_num[2] << endl;
  // cout << "contacts leg: " << (int)leg_contact_num[3] << endl;

  //---------------- LQR --------------------------

  Eigen::Matrix<double, 12, 12> A;
  Eigen::MatrixXd B;
  B.resize(12, 3 * contacts);
  Eigen::Matrix<double, 12, 12> Q;
  Eigen::MatrixXd R;
  R.resize(3 * contacts, 3 * contacts);
  Eigen::Matrix<double, 3, 3> Id;

  Eigen::Matrix<double, 3, 3> I;
  Eigen::Matrix<double, 3, 3> I_world;
  I << 15853, 0, 0, 0, 37799, 0, 0, 0, 45654;
  I = I * 1e-6;
  I_world = seResult.rBody.cast<double>() * I * seResult.rBody.transpose().cast<double>();
  double mass = 13.9;

  Eigen::Vector3d r[4];
  for (int i = 0; i < 4; i++)
  {
    r[i] = (pFoot[i] - seResult.position).cast<double>();
  }

  A.setZero();
  B.setZero();
  Q.setZero();
  R.setZero();
  Id.setIdentity();

  Q.setIdentity();
  R.setIdentity();

  A.block<3, 3>(0, 6) = Id;
  A.block<3, 3>(3, 9) = Id;

  for (size_t i = 0; i < contacts; i++)
  {
    B.block<3, 3>(6, i * 3) = cross_mat(I_world.inverse(), r[leg_contact_num[i]]);
    B.block<3, 3>(9, i * 3) = Eigen::Matrix3d::Identity() / mass;
  }

  double Q_roll = data.staticParams->Q_roll;
  double Q_pitch = data.staticParams->Q_pitch;
  double Q_yaw = data.staticParams->Q_yaw;
  double Q_x = data.staticParams->Q_x;
  double Q_y = data.staticParams->Q_y;
  double Q_z = data.staticParams->Q_z;
  double Q_w_roll = data.staticParams->Q_w_roll;
  double Q_w_pitch = data.staticParams->Q_w_pitch;
  double Q_w_yaw = data.staticParams->Q_w_yaw;
  double Q_vx = data.staticParams->Q_vx;
  double Q_vy = data.staticParams->Q_vy;
  double Q_vz = data.staticParams->Q_vz;

  Q(0, 0) = Q_roll;
  Q(1, 1) = Q_pitch;
  Q(2, 2) = Q_yaw;
  Q(3, 3) = Q_x;
  Q(4, 4) = Q_y;
  Q(5, 5) = Q_z;
  Q(6, 6) = Q_w_roll;
  Q(7, 7) = Q_w_pitch;
  Q(8, 8) = Q_w_yaw;
  Q(9, 9) = Q_vx;
  Q(10, 10) = Q_vy;
  Q(11, 11) = Q_vz;

  double alpha = data.staticParams->mpc_alpha;

  R = alpha * R;

  // cout << "A: " << A << endl;
  // cout << "B: " << B << endl;
  // cout << "Q: " << Q << endl;
  // cout << "R: " << R << endl;

  /* Solve the continuous time algebraic Ricatti equation via the Schur method */

  Eigen::MatrixXd H_LQR;
  Eigen::MatrixXd P_LQR;
  Eigen::MatrixXd s_LQR;
  H_LQR.resize(2 * 12, 2 * 12);
  P_LQR.resize(12, 12);
  s_LQR.resize(12, 1);
  // Compute the Hamiltonian and its eigenvalues/vectors
  H_LQR.block<12, 12>(0, 0) << A;
  H_LQR.block<12, 12>(0, 12) << B * R.inverse() * B.transpose();
  H_LQR.block<12, 12>(12, 0) << Q;
  H_LQR.block<12, 12>(12, 12) << -A.transpose();
  Eigen::EigenSolver<Eigen::MatrixXd> es(H_LQR);

  // Create a 2nxn matrix U=[U11;U21] containing the eigenvectors of the stable eigenvalues
  Eigen::MatrixXcd U;
  U.setZero(2 * 12, 12);
  Eigen::MatrixXcd U11;
  U11.setZero(12, 12);
  Eigen::MatrixXcd U21;
  U21.setZero(12, 12);
  Eigen::MatrixXcd P_complex;
  P_complex.setZero(12, 12);
  std::complex<double> lambda;

  // U contains eigenvectors corresponding to stable eigenvalues of the Hamiltonian
  int j = 0;
  for (int i = 0; i < 2 * 12; i++)
  {
    lambda = es.eigenvalues()[i];

    if (lambda.real() < 0 && j < 12)
    {
      U.block<2 * 12, 1>(0, j) << es.eigenvectors().col(i);
      j = j + 1;
    }
  }

  // Compute P based on U11*P = -U21;
  U11 = U.block<12, 12>(0, 0);
  U21 = U.block<12, 12>(12, 0);
  P_complex = -U21 * U11.inverse();
  P_LQR = P_complex.real();
  // cout << "P: " << P_LQR << endl;

  _pitch_des = 0;

  s_LQR(0) = 0.0 - seResult.rpy[0];
  // s_LQR(1) = 0.0 - seResult.rpy[1];
  s_LQR(1) = _pitch_des - seResult.rpy[1];
  s_LQR(2) = _yaw_des - seResult.rpy[2];

  s_LQR(3) = world_position_desired[0] - seResult.position[0];
  s_LQR(4) = world_position_desired[1] - seResult.position[1];
  s_LQR(5) = _body_height - seResult.position[2];

  s_LQR(6) = 0 - seResult.omegaWorld[0];
  s_LQR(7) = 0 - seResult.omegaWorld[1];
  s_LQR(8) = _yaw_turn_rate - seResult.omegaWorld[2];

  s_LQR(9) = v_des_world[0] - seResult.vWorld[0];
  s_LQR(10) = v_des_world[1] - seResult.vWorld[1];
  s_LQR(11) = 0 - seResult.vWorld[2];

  // cout << "error: " << s_LQR << endl;

  // Optimal control policy
  Eigen::VectorXd f_unc;
  f_unc.resize(3 * contacts, 1);
  f_unc = -R.inverse() * B.transpose() * P_LQR * s_LQR;

  Eigen::VectorXd fg;
  Eigen::Vector3d mg;
  mg.setZero();
  fg.resize(3 * contacts, 1);
  fg.setZero();

  mg << 0, 0, mass / (float)contacts * 9.81;

  for (size_t i = 0; i < contacts; i++)
  {
    fg.block<3, 1>(3 * i, 0) = -seResult.rBody.cast<double>() * mg;
  }

  f_unc += fg;

  if (std::isnan(f_unc(0)))
  {
    // cout << "-------------------------------force nan" << endl;
    f_unc = fg;
  }

  float f_max = 150;

  for (size_t i = 0; i < size(f_unc); i++)
  {
    if (f_unc[i] > f_max)
    {
      f_unc[i] = f_max;
    }

    if (f_unc[i] < -f_max)
    {
      f_unc[i] = -f_max;
    }
  }

  // float mu = 0.4;

  // for (size_t i = 0; i < contacts; i++)
  // {
  //   double Fx = 0;
  //   double Fy = 0;
  //   double Fz = 0;

  //   Fx = f_unc(3 * i + 0);
  //   Fy = f_unc(3 * i + 1);
  //   Fz = f_unc(3 * i + 2);

  //   if (Fx > mu * Fz)
  //   {
  //     Fx = mu * Fz;
  //   }
  //   if (Fx < -mu * Fz)
  //   {
  //     Fx = -mu * Fz;
  //   }

  //   if (Fy > mu * Fz)
  //   {
  //     Fy = mu * Fz;
  //   }
  //   if (Fy < -mu * Fz)
  //   {
  //     Fy = -mu * Fz;
  //   }

  //   f_unc(3 * i + 0) = Fx;
  //   f_unc(3 * i + 1) = Fy;
  //   f_unc(3 * i + 2) = Fz;
  // }

  // cout << "f: " << f_unc << endl;

  uint8_t leg_number = 0;

  Eigen::Vector3f f(0, 0, 0);

  for (size_t i = 0; i < contacts; i++)
  {
    f = f_unc.block<3, 1>(3 * i, 0).cast<float>();
    f_ff[leg_contact_num[i]] = f;
    // Fr_des[leg_contact_num[i]] = -seResult.rBody.transpose().cast<float>() * f;
    // Fr_des[leg_contact_num[i]] = -seResult.rBody.cast<float>() * f;
    Fr_des[leg_contact_num[i]] = -f;
  }

  //---------------- LQR --------------------------

  for (int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];

    if ((is_stance[foot] == 0) && !(swingState > 0))
    {
      is_stance[foot] = 1;

      data.debug->last_p_stance[foot] = ros::toMsg(pFoot[foot]);
      p_fw[foot] = pFoot[foot];

      p_fl[foot] = data.legController->datas[foot].p + data.quadruped->getHipLocation(foot);
      delta_p_bw[foot] << 0, 0, 0;
      delta_yaw[foot] = 0;
    }

    delta_p_bw[foot] += seResult.vBody * dt;
    delta_yaw[foot] += seResult.omegaBody(2) * dt;
    data.debug->last_p_local_stance[foot] = ros::toMsg(ori::rpyToRotMat(Vec3<float>(0, 0, delta_yaw[foot])) * (p_fl[foot] - delta_p_bw[foot]));

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        is_stance[foot] = 0;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);

        z_des[foot] = pFoot[foot][2];
      }

      // for visual -----------------------------------------------------------------------
      geometry_msgs::PoseStamped pose_traj;
      Vec3<float> p_des_traj(0, 0, 0);
      data.debug->leg_traj_des[foot].poses.clear();
      data.debug->leg_traj_des[foot].header.stamp = ros::Time::now();

      // 20 segment trajcetory
      for (size_t i = 0; i < 21; i++)
      {
        footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + ((1.0 - swingState) / 21.0 * (float)i), swingTimes[foot]);
        p_des_traj = footSwingTrajectories[foot].getPosition();

        pose_traj.pose.position = ros::toMsg(p_des_traj);

        data.debug->leg_traj_des[foot].poses.push_back(pose_traj);
      }
      // for visual -----------------------------------------------------------------------

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data.quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      Vec3<float> vActFootWorld = seResult.rBody.inverse() * (data.legController->datas[foot].v) + seResult.vWorld;

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

      data.debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesLeg);
      data.debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesLeg);
      data.debug->all_legs_info.leg.at(foot).p_w_act = ros::toMsg(pFoot[foot]);
      data.debug->all_legs_info.leg.at(foot).v_w_act = ros::toMsg(vActFootWorld);
      data.debug->all_legs_info.leg.at(foot).p_w_des = ros::toMsg(pDesFootWorld);
      data.debug->all_legs_info.leg.at(foot).v_w_des = ros::toMsg(vDesFootWorld);

      Eigen::Vector3f f = Eigen::Vector3f(0, 0, 0.0);
      Fr_des[foot] = -seResult.rBody * f;
      f_ff[foot] = Fr_des[foot];

      data.debug->leg_force[foot] = ros::toMsg(f_ff[foot]);

      if (!data.userParameters->use_wbc)
      {
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = Kp;
        data.legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;
      se_contactState[foot] = contactState;

      // for visual ---------------------------------------------------------
      data.debug->leg_traj_des[foot].poses.clear();
      data.debug->leg_traj_des[foot].header.stamp = ros::Time::now();
      data.debug->all_legs_info.leg[foot].swing_pf = ros::toMsg(pFoot[foot]);
      // for visual ---------------------------------------------------------

      geometry_msgs::Point point;
      point = ros::toMsg(data.legController->datas[foot].p + data.quadruped->getHipLocation(foot));
      data.debug->last_p_local_stance[foot] = point;

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      // Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> vDesFootWorld = Vec3<float>::Zero();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data.quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      Vec3<float> vActFootWorld = seResult.rBody.inverse() * (data.legController->datas[foot].v) + seResult.vWorld;

      // temporary debug
      data.debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesLeg);
      data.debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesLeg);

      if (!data.userParameters->use_wbc) // wbc off
      {
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = Kp_stance;
        data.legController->commands[foot].kdCartesian = Kd_stance;

        data.legController->commands[foot].forceFeedForward = f_ff[foot];
        data.legController->commands[foot].kdJoint = Vec3<float>(_parameters->Kd_joint_0, _parameters->Kd_joint_1, _parameters->Kd_joint_2).asDiagonal();
      }
      else
      { // Stance foot damping
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = 0. * Kp_stance;
        data.legController->commands[foot].kdCartesian = Kd_stance;
      }

      // Eigen::Vector3f f = Eigen::Vector3f(0, 0, 6.0 * 9.81);
      // // Eigen::Vector3f f(0, 0, 0);
      // f_ff[foot] = -seResult.rBody * f;
      // Fr_des[foot] = f;

      data.debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesLeg);
      data.debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesLeg);
      data.debug->all_legs_info.leg.at(foot).p_w_act = ros::toMsg(pFoot[foot]);
      data.debug->all_legs_info.leg.at(foot).v_w_act = ros::toMsg(vActFootWorld);
      data.debug->all_legs_info.leg.at(foot).p_w_des = ros::toMsg(pDesFootWorld);
      data.debug->all_legs_info.leg.at(foot).v_w_des = ros::toMsg(vDesFootWorld);

      data.debug->leg_force[foot] = ros::toMsg(f_ff[foot]);
    }
  }

  data.stateEstimator->setContactPhase(se_contactState);
  data.stateEstimator->setSwingPhase(gait->getSwingState());

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.0;
  // pBody_RPY_des[1] = 0.0;
  pBody_RPY_des[1] = _pitch_des;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  data.debug->body_info.pos_des.x = pBody_des[0];
  data.debug->body_info.pos_des.y = pBody_des[1];
  data.debug->body_info.pos_des.z = pBody_des[2];

  data.debug->body_info.vel_des.linear.x = vBody_des[0];
  data.debug->body_info.vel_des.linear.y = vBody_des[1];
  data.debug->body_info.vel_des.linear.z = vBody_des[2];

  data.debug->body_info.euler_des.x = pBody_RPY_des[0];
  data.debug->body_info.euler_des.y = pBody_RPY_des[1];
  data.debug->body_info.euler_des.z = pBody_RPY_des[2];

  data.debug->body_info.vel_des.angular.x = vBody_Ori_des[0];
  data.debug->body_info.vel_des.angular.y = vBody_Ori_des[1];
  data.debug->body_info.vel_des.angular.z = vBody_Ori_des[2];

  contact_state = gait->getContactState();
  // END of WBC Update
}

Eigen::Matrix<double, 3, 3> CMPCLocomotion::cross_mat(Eigen::Matrix<double, 3, 3> I_inv, Eigen::Matrix<double, 3, 1> r)
{
  Eigen::Matrix<double, 3, 3> cm;

  cm << 0.f, -r(2), r(1), r(2), 0.f, -r(0), -r(1), r(0), 0.f;

  return I_inv * cm;
}
