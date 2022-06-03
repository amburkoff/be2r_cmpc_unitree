#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>
#include <Utilities/utilities.h>
#include <algorithm>
#include <iostream>

#include "VisionMPCLocomotion.h"
#include "VisionMPC_interface.h"

//оригинальные параметры MPC+WBC
// #define GAIT_PERIOD 14
#define HORIZON 14

#define GAIT_PERIOD 16
// #define GAIT_PERIOD 34 //1000 Hz

//лучшие параметры для только MPC
// #define GAIT_PERIOD 18
// #define HORIZON 5

#define STEP_HEIGHT 0.06
#define BODY_HEIGHT 0.24

// #define SHOW_MPC_SOLVE_TIME

using namespace std;

////////////////////
// Controller
////////////////////

VisionMPCLocomotion::VisionMPCLocomotion(float _dt, int _iterations_between_mpc,
                                         be2r_cmpc_unitree::ros_dynamic_paramsConfig* parameters)
  : _parameters(parameters)
  , iterationsBetweenMPC(_iterations_between_mpc)
  , _body_height(_parameters->body_height)
  , _gait_period(20)
  , horizonLength(16)
  , dt(_dt)
  , trotting(
      _gait_period, Vec4<int>(0, _gait_period / 2.0, _gait_period / 2.0, 0),
      Vec4<int>(_gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0),
      "Trotting")
  , walking(
      _gait_period, Vec4<int>(2 * _gait_period / 4., 0, _gait_period / 4., 3 * _gait_period / 4.),
      Vec4<int>(0.75 * _gait_period, 0.75 * _gait_period, 0.75 * _gait_period, 0.75 * _gait_period),
      "Walking")
  , standing(_gait_period, Vec4<int>(0, 0, 0, 0),
             Vec4<int>(_gait_period, _gait_period, _gait_period, _gait_period), "Standing")
{
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Vision MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  // setup_problem(dtMPC, horizonLength, 0.4, 1200);
  vision_setup_problem(dtMPC, horizonLength, 0.4, 120); // original
  // setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
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

  //  initSparseMPC();

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
}

void VisionMPCLocomotion::initialize()
{
  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  firstRun = true;
  rpy_des.setZero();
  v_rpy_des.setZero();
}

void VisionMPCLocomotion::recompute_timing(int iterations_per_mpc)
{
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void VisionMPCLocomotion::_SetupCommand(ControlFSMData<float>& data)
{
  _body_height = _parameters->body_height;

  float x_vel_cmd, y_vel_cmd;
  float filter(0.1);

  _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];
  x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];
  y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];
  // _yaw_turn_rate = 0;
  // x_vel_cmd = 0;
  // y_vel_cmd = 0;

  _x_vel_des = _x_vel_des * (1 - filter) + x_vel_cmd * filter;
  _y_vel_des = _y_vel_des * (1 - filter) + y_vel_cmd * filter;

  //  _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  _yaw_des += dt * _yaw_turn_rate;
  _roll_des = 0.;
  _pitch_des = 0.;

  // Update PD coefs
  Kp = Vec3<float>(_parameters->Kp_cartesian_0, _parameters->Kp_cartesian_1,
                   _parameters->Kp_cartesian_2)
         .asDiagonal();
  Kp_stance = Kp;

  Kd = Vec3<float>(_parameters->Kd_cartesian_0, _parameters->Kd_cartesian_1,
                   _parameters->Kd_cartesian_2)
         .asDiagonal();
  Kd_stance = Kd;
}

void VisionMPCLocomotion::run(ControlFSMData<float>& data, const Vec3<float>& vel_cmd,
                              const grid_map::GridMap& height_map,
                              const grid_map::GridMap& height_map_raw)
{
  // Command Setup
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;
  // if (gaitNumber >= 10)
  // {
  //   gaitNumber -= 10;
  //   omniMode = true;
  // }

  auto& seResult = data._stateEstimator->getResult();

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

  // cout << "[VisionMPCLocomotion] get res done" << endl;

  // pick gait
  Gait* gait = &trotting;
  if (gaitNumber == 10)
  {
    gait = &walking;
  }
  current_gait = gaitNumber;

  //  gait->restoreDefaults();
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  //  gait->earlyContactHandle(seResult.contactSensor, iterationsBetweenMPC, iterationCounter);
  //  std::cout << "iterationCounter " << iterationCounter << std::endl;

  recompute_timing(default_iterations_between_mpc);

  if (_body_height < 0.02)
  {
    _body_height = 0.24;
  }

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  //                      Pitch compensation
  static Vec3<float> pDesFootWorldStance[4] = { pFoot[0], pFoot[1], pFoot[2], pFoot[3] };

  // p front mid, p back mid
  Vec3<float> p_fm = (pDesFootWorldStance[0] + pDesFootWorldStance[1]) / 2;
  Vec3<float> p_bm = (pDesFootWorldStance[2] + pDesFootWorldStance[3]) / 2;
  float des_pitch = 0;
  float des_roll = 0;

  // XZ plane
  float L_xz = sqrt((p_fm(2) - p_bm(2)) * (p_fm(2) - p_bm(2)) + (0.1805 * 2) * (0.1805 * 2));

  if (abs(L_xz) < 0.0001)
  {
    des_pitch = 0;
  }
  else
  {
    des_pitch = des_pitch * (1 - 0.7) - 2 * asin((p_fm(2) - p_bm(2)) / (L_xz)) * 0.7;
  }

  // put to target
  _pitch_des = des_pitch;
  data.debug->all_legs_info.leg[0].force_raw = des_pitch;
  //                      Pitch compensation

  // Integral-esque pitche and roll compensation
  if (fabs(v_robot[0]) > .2) // avoid dividing by zero
  {
    rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
  }
  if (fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
  }

  // cout << "cmpc se r: " << seResult.rpy[0] << " se p: " << seResult.rpy[1] <<
  // " se y: " << seResult.rpy[2] << endl;

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber != 8); // turn off for pronking

  for (int i = 0; i < 4; i++)
  {
    pFoot[i] =
      seResult.position + seResult.rBody.transpose() *
                            (data._quadruped->getHipLocation(i) + data._legController->datas[i].p);
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
    world_position_desired[2] = seResult.rpy[2];

    for (int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(_parameters->Swing_traj_height);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
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
  // float interleave_gain = -0.13;
  float interleave_gain = -0.2;
  // float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  double swing_height = 0;

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

    Vec3<float> offset(0, side_sign[i] * data._quadruped->_abadLinkLength, 0);

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);

    Vec3<float> pYawCorrected =
      coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position +
                     seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

    //+ seResult.vWorld * swingTimeRemaining[i];

    // float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    // Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
    float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time +
                    .03f * (seResult.vWorld[0] - v_des_world[0]) +
                    (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate);

    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
                    .03f * (seResult.vWorld[1] - v_des_world[1]) +
                    (0.5f * seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * _yaw_turn_rate);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    Pf[2] = 0;
    _updateFoothold(Pf, seResult.position, height_map, height_map_raw, i);
    footSwingTrajectories[i].setFinalPosition(Pf);
  }

  // calc gait
  iterationCounter++;

  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();

  updateMPCIfNeeded(mpcTable, data);

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0, 0, 0, 0);

  // ROS_INFO_STREAM("is contact: " << se_contactState(0));

  // static bool is_stance[4] = {0, 0, 0, 0};

  static geometry_msgs::PoseStamped pose[4];

  for (int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
        swing_height = _updateTrajHeight(foot);

        footSwingTrajectories[foot].setHeight(swing_height);
        // is_stance[foot] = 0;

        geometry_msgs::PoseStamped Emptypose;
        pose[foot] = Emptypose;
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) -
                            data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      if (foot == 3 || foot == 2)
      {
        std::cout << "Foot z P0 = " << footSwingTrajectories[foot].getInitialPosition()[2]
                  << std::endl;
        std::cout << "Foot z PF = " << footSwingTrajectories[foot].getFinalPosition()[2]
                  << std::endl;
      }

      // temporary debug
      data.debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesLeg);
      data.debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesLeg);

      pose[foot].pose.position.x = pDesFootWorld.x();
      pose[foot].pose.position.y = pDesFootWorld.y();
      pose[foot].pose.position.z = pDesFootWorld.z();

      pose[foot].pose.orientation.x = 0;
      pose[foot].pose.orientation.y = 0;
      pose[foot].pose.orientation.z = 0;
      pose[foot].pose.orientation.w = 1;

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

      if (!data.userParameters->use_wbc)
      {
        // Update leg control command regardless of the usage of WBIC
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
      }

      std::string names[4] = { "FR_hip", "FL_hip", "RR_hip", "RL_hip" };

      geometry_msgs::Point p1, p2;
      // start point
      p1.x = 0;
      p1.y = 0;
      p1.z = 0;
      // finish point
      p2.x = 0;
      p2.y = 0;
      p2.z = 0;
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;
      pDesFootWorldStance[foot] = pFoot[foot];

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      // Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> vDesFootWorld = Vec3<float>::Zero();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) -
                            data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // cout << "Foot " << foot << " relative velocity desired: " <<
      // vDesLeg.transpose() << "\n";

      // temporary debug
      data.debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesLeg);
      data.debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesLeg);

      if (!data.userParameters->use_wbc) // wbc off
      {
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint =
          Vec3<float>(_parameters->Kd_joint_0, _parameters->Kd_joint_1, _parameters->Kd_joint_2)
            .asDiagonal();

        //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd,
        //                                          0); todo removed
        // hw_i->leg_controller->leg_commands[foot].tau_ff +=
        // 0*footSwingController[foot]->getTauFF();
      }
      else
      { // Stance foot damping
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = 0. * Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
      }
      //            cout << "Foot " << foot << " force: " <<
      //            f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      std::string names[4] = { "FR_hip", "FL_hip", "RR_hip", "RL_hip" };
      geometry_msgs::Point p1, p2;
      // start point
      p1.x = pDesLeg[0];
      p1.y = pDesLeg[1];
      p1.z = pDesLeg[2];
      // finish point
      float koef = 500;
      p2.x = pDesLeg[0] + (-f_ff[foot][0] / koef);
      p2.y = pDesLeg[1] + (-f_ff[foot][1] / koef);
      p2.z = pDesLeg[2] + (-f_ff[foot][2] / koef);
    }
  }

  data._stateEstimator->setContactPhase(se_contactState);
  data._stateEstimator->setSwingPhase(gait->getSwingState());

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

void VisionMPCLocomotion::updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data)
{
  // iterationsBetweenMPC = 30;
  if ((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<float> v_des_world = seResult.rBody.transpose() * v_des_robot;
    // float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    // printf("Position error: %.3f, integral %.3f\n", pxy_err[0],
    // x_comp_integral);

    // Stand gait
    if (current_gait == 4)
    {
      float trajInitial[12] = {
        _roll_des,
        _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
        (float)stand_traj[5] /*+(float)stateCommand->data.stateDes[11]*/,
        (float)stand_traj[0] /*+(float)fsm->main_control_settings.p_des[0]*/,
        (float)stand_traj[1] /*+(float)fsm->main_control_settings.p_des[1]*/,
        (float)_body_height /*fsm->main_control_settings.p_des[2]*/,
        0,
        0,
        0,
        0,
        0,
        0
      };

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

    solveDenseMPC(mpcTable, data);

    // printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }
}

void VisionMPCLocomotion::solveDenseMPC(int* mpcTable, ControlFSMData<float>& data)
{
  auto seResult = data._stateEstimator->getResult();

  // float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};

  // float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};
  // //original
  float Q[12] = { 2.5, 2.5, 10, 50, 50, 100, 0, 0, 0.5, 0.2, 0.2, 0.1 };

  // float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
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

  // printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

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
  dtMPC = dt * iterationsBetweenMPC;
  vision_setup_problem(dtMPC, horizonLength, 0.4, 120);
  // setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);

  if (vxy[0] > 0.3 || vxy[0] < -0.3)
  {
    // x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC /
    // vxy[0];
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  // printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho, _parameters->jcqp_sigma,
                         _parameters->jcqp_alpha, _parameters->jcqp_terminate,
                         _parameters->use_jcqp);
  // t1.stopPrint("Setup MPC");
  // printf("MPC Setup time %f ms\n", t1.getMs());

  Timer t2;
  // cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);
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

float VisionMPCLocomotion::_updateTrajHeight(size_t foot)
{
  if (foot == 2 || foot == 3)
    return _parameters->Swing_traj_height;

  double h = 0;

  h = (footSwingTrajectories[foot].getFinalPosition()(2) -
       footSwingTrajectories[foot].getInitialPosition()(2)) *
      0.5;
  // Saturate h
  h = std::clamp(h, -_parameters->Swing_traj_height, _parameters->Swing_traj_height);
  double out = _parameters->Swing_traj_height + h;
  out = std::clamp(out, -MAX_STEP_HEIGHT, MAX_STEP_HEIGHT);
  return out;
}

void VisionMPCLocomotion::_updateFoothold(Vec3<float>& foot, const Vec3<float>& body_pos,
                                          const grid_map::GridMap& height_map,
                                          const grid_map::GridMap& height_map_raw, int leg)
{
  static bool leg2 = 0;
  static bool leg3 = 0;
  // Положение лапы в СК тела
  //  Vec3<float> scale(1.2, 1, 1);
  Vec3<float> local_pf = foot - body_pos;
  //  Vec3<float> local_pf_scaled = foot.cwiseProduct(scale) - body_pos;
  //  std::cout << "Pf in base frame: " << std::endl << local_pf << std::endl;

  // Координаты центра карты
  int row_idx_half = height_map.getSize()(0) / 2;
  int col_idx_half = height_map.getSize()(1) / 2;
  //  std::cout << "Heightmap center (x y) : " << row_idx_half << " " << col_idx_half << std::endl;

  // Минус для преобразования координат
  int x_idx = col_idx_half - floor(local_pf[0] / grid_size);
  int y_idx = row_idx_half - floor(local_pf[1] / grid_size);

  int x_idx_selected = x_idx;
  int y_idx_selected = y_idx;

  _IdxMapChecking(local_pf, x_idx, y_idx, x_idx_selected, y_idx_selected, height_map_raw, leg);

  // Минус для преобразования координат
  foot[0] = -(x_idx_selected - row_idx_half) * grid_size + body_pos[0];
  foot[1] = -(y_idx_selected - col_idx_half) * grid_size + body_pos[1];
  auto h = height_map.at("elevation", Eigen::Array2i(x_idx_selected, y_idx_selected));
  //  if (leg == 2 || leg == 3)
  //  {
  //    if (footSwingTrajectories[leg].getInitialPosition()[2] > 0.1)
  //      _data->debug->z_offset += h;
  //  }
  //  if (leg == 3 && pFoot[leg][2] > 0.1 && footSwingTrajectories[leg].getInitialPosition()[2] >
  //  0.1)
  //  {
  //    leg3 = true;
  //    if (leg3 && leg2)
  //    {
  //      _data->debug->z_offset += h;
  //      leg3 = false;
  //      leg2 = false;
  //    }
  //  }
  //  if (leg == 2 && pFoot[leg][2] > 0.1 && footSwingTrajectories[leg].getInitialPosition()[2] >
  //  0.1)
  //  {
  //    leg2 = true;
  //    if (leg3 && leg2)
  //    {
  //      _data->debug->z_offset += h;
  //      leg2 = false;
  //      leg3 = false;
  //    }
  //  }
  if (_data->_stateEstimator->getResult().position(0) > 0.6)
  {
    double x = _data->_stateEstimator->getResult().position(0) - 0.6;
    _data->debug->z_offset = x * 0.42;
  }
  h -= _data->debug->z_offset;
  foot[2] = std::isnan(h) ? 0. : h;
  //  if (leg == 3 || leg == 2)
  //    std::cout << "Foot z PF = " << foot[2] << std::endl;
}

void VisionMPCLocomotion::_IdxMapChecking(Vec3<float>& Pf, int x_idx, int y_idx,
                                          int& x_idx_selected, int& y_idx_selected,
                                          const grid_map::GridMap& height_map, int leg)
{
  grid_map::Index center(x_idx, y_idx);
  // std::cout << " Leg position (x,y) " << Pf[0] << " " << Pf[1] << std::endl;
  double radius = 0.04;
  // std::cout << "Normal is " << height_map.at("normal_vectors_z", Eigen::Array2i(x_idx, y_idx)) <<
  // std::endl;
  for (grid_map_utils::SpiralIterator iterator(height_map, center, radius); !iterator.isPastEnd();
       ++iterator)
  {
    auto norm_z = height_map.at("normal_vectors_z", *iterator);
    // If cell is flat
    if (!std::isnan(norm_z) && norm_z > 0.98)
    {
      x_idx_selected = (*iterator)(0);
      y_idx_selected = (*iterator)(1);
      if ((x_idx != x_idx_selected) && (y_idx != y_idx_selected))
        std::cout << "Edit footstep from ( " << x_idx << " " << y_idx << ") to ( " << x_idx_selected
                  << " " << y_idx_selected << " )" << std::endl;
      return;
    }
  }
}
