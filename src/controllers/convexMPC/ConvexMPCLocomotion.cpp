#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>
#include <Utilities/utilities.h>
#include <iostream>

#include "ConvexMPCLocomotion.h"
#include "GraphSearch.h"
#include "convexMPC_interface.h"

#include "Gait.h"

// оригинальные параметры MPC+WBC
//  #define GAIT_PERIOD 14
#define HORIZON 16

#define GAIT_PERIOD 18
// #define GAIT_PERIOD 34 //1000 Hz

// лучшие параметры для только MPC
//  #define GAIT_PERIOD 18
//  #define HORIZON 5

#define STEP_HEIGHT 0.06
#define BODY_HEIGHT 0.24

// #define SHOW_MPC_SOLVE_TIME

using namespace std;

////////////////////
// Controller
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int iterations_between_mpc, ControlFSMData<float>* data)
  : _fsm_data(data),
    _iterationsBetweenMPC(iterations_between_mpc),
    _dyn_params(data->userParameters),
    _gait_period(_dyn_params->gait_period),
    horizonLength(_fsm_data->staticParams->horizon),
    dt(_dt),
    trotting(_gait_period, Vec4<int>(0, _gait_period / 2.0, _gait_period / 2.0, 0), Vec4<int>(_gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0), "Trotting"),
    bounding(_gait_period, Vec4<int>(5, 5, 0, 0), Vec4<int>(4, 4, 4, 4), "Bounding"),
    pronking(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(8, 8, 8, 8), "Pronking"),
    jumping(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(2, 2, 2, 2), "Jumping"),
    galloping(_gait_period, Vec4<int>(0, 2, 7, 9), Vec4<int>(4, 4, 4, 4), "Galloping"),
    standing(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(_gait_period, _gait_period, _gait_period, _gait_period), "Standing"),
    trotRunning(_gait_period, Vec4<int>(0, 5, 5, 0), Vec4<int>(4, 4, 4, 4), "Trot Running"),
    walking(_gait_period, Vec4<int>(2 * _gait_period / 4., 0, _gait_period / 4., 3 * _gait_period / 4.), Vec4<int>(0.75 * _gait_period, 0.75 * _gait_period, 0.75 * _gait_period, 0.75 * _gait_period), "Walking"),
    walking2(_gait_period, Vec4<int>(0, 5, 5, 0), Vec4<int>(7, 7, 7, 7), "Walking2"),
    pacing(_gait_period, Vec4<int>(5, 0, 5, 0), Vec4<int>(5, 5, 5, 5), "Pacing"),
    random(_gait_period, Vec4<int>(9, 13, 13, 9), 0.4, "Flying nine thirteenths trot"),
    random2(_gait_period, Vec4<int>(8, 16, 16, 8), 0.5, "Double Trot")
{
  ROS_WARN_STREAM("Current gait period " << _dyn_params->gait_period);
  dtMPC = dt * _iterationsBetweenMPC;
  default_iterations_between_mpc = _iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, _iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.4, 120, _fsm_data->quadruped->_whole_mass); // original (3d arg prev: 1200, 650)
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  _yaw_des = 0;
  _pitch_des = 0.;

  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  initSparseMPC();

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
}

void ConvexMPCLocomotion::initialize()
{
  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc)
{
  _iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float>& data)
{
  _body_height = _dyn_params->body_height;

  float x_vel_cmd, y_vel_cmd;
  float filter(0.1);

  _yaw_turn_rate = data.gamepad_command->right_stick_analog[0];
  x_vel_cmd = data.gamepad_command->left_stick_analog[1];
  y_vel_cmd = data.gamepad_command->left_stick_analog[0];

  _yaw_turn_rate *= data.staticParams->max_turn_rate;
  x_vel_cmd *= data.staticParams->max_vel_x;
  y_vel_cmd *= data.staticParams->max_vel_y;

  _x_vel_des = _x_vel_des * (1 - filter) + x_vel_cmd * filter;
  _y_vel_des = _y_vel_des * (1 - filter) + y_vel_cmd * filter;

  _yaw_des = data.stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  _roll_des = 0.;
  _pitch_des = 0.;

  // Update PD coefs
  Kp = Vec3<float>(_dyn_params->Kp_cartesian_0, _dyn_params->Kp_cartesian_1, _dyn_params->Kp_cartesian_2).asDiagonal();
  Kp_stance = Kp;

  Kd = Vec3<float>(_dyn_params->Kd_cartesian_0, _dyn_params->Kd_cartesian_1, _dyn_params->Kd_cartesian_2).asDiagonal();
  Kd_stance = Kd;
}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<float>& data)
{
  bool omniMode = false;

  // Command Setup
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;

  auto& seResult = data.stateEstimator->getResult();

  // Check if transition to standing
  if (((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // cout << "[ConvexMPCLocomotion] get res done" << endl;

  // pick gait
  Gait* gait = &trotting;
  if (gaitNumber == 1)
  {
    gait = &bounding;
  }
  else if (gaitNumber == 2)
  {
    gait = &pronking;
  }
  else if (gaitNumber == 3)
  {
    gait = &jumping;
  }
  else if (gaitNumber == 4)
  {
    gait = &standing;
  }
  else if (gaitNumber == 5)
  {
    gait = &trotRunning;
  }
  else if (gaitNumber == 6)
  {
    gait = &galloping;
  }
  // else if (gaitNumber == 7)
  // {
  //   gait = &random2;
  // }
  else if (gaitNumber == 8)
  {
    gait = &pacing;
  }
  else if (gaitNumber == 10)
  {
    gait = &walking;
  }
  else if (gaitNumber == 11)
  {
    gait = &walking2;
  }
  current_gait = gaitNumber;

  // gait->updatePeriod(_dyn_params->gait_period);
  //  gait->restoreDefaults();
  gait->setIterations(_iterationsBetweenMPC, iterationCounter);
  //  gait->earlyContactHandle(seResult.contactSensor, _iterationsBetweenMPC, iterationCounter);

  recompute_timing(default_iterations_between_mpc);

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  //                      Pitch compensation
  static Vec3<float> pDesFootWorldStance[4] = { pFoot[0], pFoot[1], pFoot[2], pFoot[3] };

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
    // world_position_desired[2] = seResult.rpy[2];
    world_position_desired[2] = _body_height;
    _yaw_des = seResult.rpy[2];

    for (int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(_dyn_params->Swing_traj_height);

      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_ps.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_ps.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_ps.z = pFoot[i](2);

      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_pf.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_pf.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_pf.z = pFoot[i](2);
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

    footSwingTrajectories[i].setHeight(_dyn_params->Swing_traj_height);

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

    float pfx_rel = seResult.vWorld[0] * (.5 + _dyn_params->cmpc_bonus_swing) * stance_time + .03f * (seResult.vWorld[0] - v_des_world[0]) + (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate);

    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC + .03f * (seResult.vWorld[1] - v_des_world[1]) + (0.5f * seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * _yaw_turn_rate);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    Pf[2] = 0.0;

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

  updateMPCIfNeeded(mpcTable, data, omniMode);

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

      // foot position in world frame at contanct
      pDesFootWorldStance[foot] = pFoot[foot];
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
      }

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

      if (!data.userParameters->use_wbc)
      {
        // Update leg control command regardless of the usage of WBIC
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = Kp;
        data.legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;
      pDesFootWorldStance[foot] = pFoot[foot];

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
        data.legController->commands[foot].kdJoint = Vec3<float>(_dyn_params->Kd_joint_0, _dyn_params->Kd_joint_1, _dyn_params->Kd_joint_2).asDiagonal();
      }
      else
      { // Stance foot damping
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = 0. * Kp_stance;
        data.legController->commands[foot].kdCartesian = Kd_stance;
      }

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

  pBody_RPY_des[0] = 0.;
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

template<>
void ConvexMPCLocomotion::run(ControlFSMData<double>& data)
{
  (void)data;
  printf("call to old CMPC with double!\n");
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode)
{
  // _iterationsBetweenMPC = 30;
  if ((iterationCounter % _iterationsBetweenMPC) == 0)
  {
    auto seResult = data.stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    // float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    // printf("Position error: %.3f, integral %.3f\n", pxy_err[0],
    // x_comp_integral);

    // Stand gait
    if (current_gait == 4)
    {
      float trajInitial[12] = { _roll_des, _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/, (float)stand_traj[5] /*+(float)stateCommand->data.stateDes[11]*/, (float)stand_traj[0] /*+(float)fsm->main_control_settings.p_des[0]*/, (float)stand_traj[1] /*+(float)fsm->main_control_settings.p_des[1]*/, (float)_body_height /*fsm->main_control_settings.p_des[2]*/, 0, 0, 0, 0, 0, 0 };

      for (int i = 0; i < horizonLength; i++)
        for (int j = 0; j < 12; j++)
          trajAll[12 * i + j] = trajInitial[j];
    }
    else
    {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if (xStart - p[0] > max_pos_error)
        xStart = p[0] + max_pos_error;
      if (p[0] - xStart > max_pos_error)
        xStart = p[0] - max_pos_error;

      if (yStart - p[1] > max_pos_error)
        yStart = p[1] + max_pos_error;
      if (p[1] - yStart > max_pos_error)
        yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = { (float)rpy_comp[0], // 0
                                (float)rpy_comp[1], // 1
                                _yaw_des,           // 2
                                // yawStart,    // 2
                                xStart,              // 3
                                yStart,              // 4
                                (float)_body_height, // 5
                                0,                   // 6
                                0,                   // 7
                                _yaw_turn_rate,      // 8
                                v_des_world[0],      // 9
                                v_des_world[1],      // 10
                                0 };                 // 11

      for (int i = 0; i < horizonLength; i++)
      {
        for (int j = 0; j < 12; j++)
          trajAll[12 * i + j] = trajInitial[j];

        if (i == 0) // start at current position  TODO consider not doing this
        {
          // trajAll[3] = hw_i->state_estimator->se_pBody[0];
          // trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }

    Timer solveTimer;

    if (_dyn_params->cmpc_use_sparse > 0.5)
    {
      solveSparseMPC(mpcTable, data);
    }
    else
    {
      solveDenseMPC(mpcTable, data);
    }
    // printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }
}

void ConvexMPCLocomotion::solveDenseMPC(int* mpcTable, ControlFSMData<float>& data)
{
  auto seResult = data.stateEstimator->getResult();

  // original
  float Q[12] = { 0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1 };

  float roll = seResult.rpy[0];
  float pitch = seResult.rpy[1];
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  // float alpha = 4e-7; // make setting eventually: DH
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

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  // Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * _iterationsBetweenMPC;
  setup_problem(dtMPC, horizonLength, 0.4, 120, _fsm_data->quadruped->_whole_mass);
  // setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);

  if (vxy[0] > 0.3 || vxy[0] < -0.3)
  {
    // x_comp_integral += _dyn_params->cmpc_x_drag * pxy_err[0] * dtMPC /
    // vxy[0];
    x_comp_integral += _dyn_params->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  // printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_dyn_params->jcqp_max_iter, _dyn_params->jcqp_rho, _dyn_params->jcqp_sigma, _dyn_params->jcqp_alpha, _dyn_params->jcqp_terminate, _dyn_params->use_jcqp);
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
      f[axis] = get_solution(leg * 3 + axis);

    // printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}

void ConvexMPCLocomotion::solveSparseMPC(int* mpcTable, ControlFSMData<float>& data)
{
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  (void)data;
  auto seResult = data.stateEstimator->getResult();

  std::vector<ContactState> contactStates;
  for (int i = 0; i < horizonLength; i++)
  {
    contactStates.emplace_back(mpcTable[i * 4 + 0], mpcTable[i * 4 + 1], mpcTable[i * 4 + 2], mpcTable[i * 4 + 3]);
  }

  for (int i = 0; i < horizonLength; i++)
  {
    for (u32 j = 0; j < 12; j++)
    {
      _sparseTrajectory[i][j] = trajAll[i * 12 + j];
    }
  }

  Vec12<float> feet;
  for (u32 foot = 0; foot < 4; foot++)
  {
    for (u32 axis = 0; axis < 3; axis++)
    {
      feet[foot * 3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for (u32 foot = 0; foot < 4; foot++)
  {
    Vec3<float> force(resultForce[foot * 3], resultForce[foot * 3 + 1], resultForce[foot * 3 + 2]);
    // printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}

void ConvexMPCLocomotion::initSparseMPC()
{
  Mat3<double> baseInertia;
  baseInertia << 0.07, 0, 0, 0, 0.26, 0, 0, 0, 0.242;
  double mass = 9;
  double maxForce = 120;

  std::vector<double> dtTraj;
  for (int i = 0; i < horizonLength; i++)
  {
    dtTraj.push_back(dtMPC);
  }

  Vec12<double> weights;
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
  // weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseCMPC.setFriction(1.0);
  // _sparseCMPC.setFriction(0.4);
  _sparseCMPC.setWeights(weights, 4e-5);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}
