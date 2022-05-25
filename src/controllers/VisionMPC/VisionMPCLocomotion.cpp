#include <Utilities/Utilities_print.h>
#include <iostream>

#include "VisionMPCLocomotion.h"
#include "VisionMPC_interface.h"

///////////////
// GAIT
///////////////
VisionGait::VisionGait(int nMPC_segments, Vec4<int> offsets, Vec4<int> durations, const std::string& name)
  : _offsets(offsets.array())
  , _durations(durations.array())
  , _nIterations(nMPC_segments)
{
  _mpc_table = new int[nMPC_segments * 4];

  _offsetsFloat = offsets.cast<float>() / (float)nMPC_segments;
  _durationsFloat = durations.cast<float>() / (float)nMPC_segments;
  std::cout << "VisionGait " << name << "\n";
  std::cout << "nMPC_segments    : " << _nIterations << "\n";
  std::cout << "offsets (int)    : " << _offsets.transpose() << "\n";
  std::cout << "durations (int)  : " << _durations.transpose() << "\n";
  std::cout << "offsets (float)  : " << _offsetsFloat.transpose() << "\n";
  std::cout << "durations (float): " << _durationsFloat.transpose() << "\n";
  std::cout << "\n\n";

  _stance = durations[0];
  _swing = nMPC_segments - durations[0];
}

VisionGait::~VisionGait() { delete[] _mpc_table; }

Vec4<float> VisionGait::getContactState()
{
  //  Array4f offset = _offsetsFloat;
  //  for (int i = 0; i < 4; i++)
  //  {
  //    if (offset[i] < 0)
  //      offset[i] += 1.f;
  //  }

  Array4f progress = _phase - _offsetsFloat;

  for (int i = 0; i < 4; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.;
    if (progress[i] > _durationsFloat[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsFloat[i];
    }
  }

  return progress.matrix();
}

Vec4<float> VisionGait::getSwingState()
{
  Array4f swing_offset = _offsetsFloat + _durationsFloat;
  for (int i = 0; i < 4; i++)
    if (swing_offset[i] > 1)
      swing_offset[i] -= 1.;
  Array4f swing_duration = 1. - _durationsFloat;

  Array4f progress = _phase - swing_offset;

  for (int i = 0; i < 4; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.f;
    if (progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  return progress.matrix();
}

int* VisionGait::mpc_gait()
{
  for (int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    for (int j = 0; j < 4; j++)
    {
      if (progress[j] < 0)
        progress[j] += _nIterations;
      if (progress[j] < _durations[j])
        _mpc_table[i * 4 + j] = 1;
      else
        _mpc_table[i * 4 + j] = 0;
    }
  }

  return _mpc_table;
}

void VisionGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float)(iterationsPerMPC * _nIterations);
}

////////////////////
// Controller
////////////////////

VisionMPCLocomotion::VisionMPCLocomotion(float _dt, int _iterations_between_mpc,
                                         be2r_cmpc_unitree::ros_dynamic_paramsConfig* parameters)
  : _parameters(parameters)
  , iterationsBetweenMPC(_iterations_between_mpc)
  , _body_height(_parameters->body_height)
  , _gait_period(_parameters->gait_period)
  , horizonLength(16)
  , dt(_dt)
  , trotting(_gait_period, Vec4<int>(0, _gait_period / 2.0, _gait_period / 2.0, 0),
             Vec4<int>(_gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0), "Trotting")
  , bounding(_gait_period, Vec4<int>(5, 5, 0, 0), Vec4<int>(3, 3, 3, 3), "Bounding")
  , pronking(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(4, 4, 4, 4), "Pronking")
  , galloping(_gait_period, Vec4<int>(0, 2, 7, 9), Vec4<int>(3, 3, 3, 3), "Galloping")
  , standing(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(10, 10, 10, 10), "Standing")
  , trotRunning(_gait_period, Vec4<int>(0, 5, 5, 0), Vec4<int>(3, 3, 3, 3), "Trot Running")
{
  dtMPC = dt * iterationsBetweenMPC;
  printf("[Vision MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  vision_setup_problem(dtMPC, horizonLength, 0.4, 120);
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for (int i = 0; i < 4; i++)
    firstSwing[i] = true;
}

void VisionMPCLocomotion::initialize()
{
  for (int i = 0; i < 4; i++)
    firstSwing[i] = true;
  firstRun = true;
  rpy_des.setZero();
  v_rpy_des.setZero();
}
void VisionMPCLocomotion::_updateFoothold(Vec3<float>& foot, const Vec3<float>& body_pos,
                                          const grid_map::GridMap& height_map, const grid_map::GridMap& height_map_raw)
{
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
  //  std::cout << "Heightmap index (x y) : " << x_idx << " " << y_idx << std::endl;

  int x_idx_selected = x_idx;
  int y_idx_selected = y_idx;

  _IdxMapChecking(local_pf, x_idx, y_idx, x_idx_selected, y_idx_selected, height_map_raw);

  // Минус для преобразования координат
  foot[0] = -(x_idx_selected - row_idx_half) * grid_size + body_pos[0];
  foot[1] = -(y_idx_selected - col_idx_half) * grid_size + body_pos[1];
  auto h = height_map.at("elevation", Eigen::Array2i(x_idx_selected, y_idx_selected));
  foot[2] = std::isnan(h) ? 0. : h;
}

void VisionMPCLocomotion::_IdxMapChecking(Vec3<float>& Pf, int x_idx, int y_idx, int& x_idx_selected,
                                          int& y_idx_selected, const grid_map::GridMap& height_map)
{
  grid_map::Position center(Pf[0], Pf[1]);
  double radius = 0.1;

  for (grid_map::SpiralIterator iterator(height_map, center, radius); !iterator.isPastEnd(); ++iterator)
  {
    if (height_map.at("normal_vectors_z", *iterator) < 0.5 && height_map.at("normal_vectors_z", *iterator) > 0.01)
      std::cout << "Normal is " << height_map.at("normal_vectors_z", *iterator) << std::endl;
  }
}

void VisionMPCLocomotion::_updateParams(ControlFSMData<float>& data)
{
  _body_height = _parameters->body_height;

  // Update PD coefs
  Kp = Vec3<float>(_parameters->Kp_cartesian_0, _parameters->Kp_cartesian_1, _parameters->Kp_cartesian_2).asDiagonal();
  Kp_stance = Kp;

  Kd = Vec3<float>(_parameters->Kd_cartesian_0, _parameters->Kd_cartesian_1, _parameters->Kd_cartesian_2).asDiagonal();
  Kd_stance = Kd;
}

void VisionMPCLocomotion::run(ControlFSMData<float>& data, const Vec3<float>& vel_cmd,
                              const grid_map::GridMap& height_map, const grid_map::GridMap& height_map_raw)
{
  _updateParams(data);
  gaitNumber = data.userParameters->cmpc_gait;
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

  // pick gait
  VisionGait* gait = &trotting;
  //  if (gaitNumber == 1)
  //    gait = &bounding;
  //  else if (gaitNumber == 2)
  //    gait = &pronking;
  //  else if (gaitNumber == 3)
  //    gait = &galloping;
  //  else if (gaitNumber == 4)
  //    gait = &standing;
  //  else if (gaitNumber == 5)
  //    gait = &trotRunning;
  current_gait = gaitNumber;
  //  std::cout << " Current gait" << current_gait << std::endl;

  // integrate position setpoint
  v_des_world[0] = vel_cmd[0];
  v_des_world[1] = vel_cmd[1];
  v_des_world[2] = 0.;
  v_rpy_des[2] = vel_cmd[2];
  rpy_des[2] += dt * v_rpy_des[2];
  Vec3<float> v_robot = seResult.vWorld;

  // pretty_print(v_des_world, std::cout, "v des world");

  // Integral-esque pitche and roll compensation
  if (fabs(v_robot[0]) > .2)
  { // avoid dividing by zero
    rpy_int[1] += dt * (rpy_des[1] - seResult.rpy[1]) / v_robot[0];
  }
  if (fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt * (rpy_des[0] - seResult.rpy[0]) / v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber != 8); // turn off for pronking

  for (int i = 0; i < 4; i++)
  {
    pFoot[i] = seResult.position +
               seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + data._legController->datas[i].p);
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
  swingTimes[0] = dtMPC * gait->_swing;
  swingTimes[1] = dtMPC * gait->_swing;
  swingTimes[2] = dtMPC * gait->_swing;
  swingTimes[3] = dtMPC * gait->_swing;

  float side_sign[4] = { -1, 1, -1, 1 };

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

    // Swing Height
    footSwingTrajectories[i].setHeight(_parameters->Swing_traj_height);
    Vec3<float> offset(0, side_sign[i] * data._quadruped->_abadLinkLength, 0);

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);
    Vec3<float> pYawCorrected =
      coordinateRotation(CoordinateAxis::Z, -v_rpy_des[2] * gait->_stance * dtMPC / 2) * pRobotFrame;

    Vec3<float> des_vel = seResult.rBody * v_des_world;
    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    float pfx_rel = seResult.vWorld[0] * .5 * gait->_stance * dtMPC + .03f * (seResult.vWorld[0] - v_des_world[0]) +
                    (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[1] * v_rpy_des[2]);

    float pfy_rel = seResult.vWorld[1] * .5 * gait->_stance * dtMPC + .03f * (seResult.vWorld[1] - v_des_world[1]) +
                    (0.5f * seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * v_rpy_des[2]);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;

    _updateFoothold(Pf, seResult.position, height_map, height_map_raw);
    Pf[2] = Pf[2] >= 1e-3 ? Pf[2] : 0.; // Только положительные
    _fin_foot_loc[i] = Pf;
    //    std::cout << "Foot [" << i << "] target z is " << Pf[2] << std::endl;
    footSwingTrajectories[i].setFinalPosition(Pf);
  }
  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  iterationCounter++;

  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->mpc_gait();
  updateMPCIfNeeded(mpcTable, data);

  Vec4<float> se_contactState(0, 0, 0, 0);

  for (int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    // foot is in SWING
    if (swingState > 0)
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }
      //      footSwingTrajectories[foot].setHeight(_fin_foot_loc[foot][2] + 0.04); // change to
      //      hardcode
      //      float step_height = _fin_foot_loc[foot][2] + 0.04 <= MAX_STEP_HEIGHT
      //                            ? _fin_foot_loc[foot][2] + 0.04
      //                            : MAX_STEP_HEIGHT;
      // TODO: прибавлять к высоте траектории разницу в высоте между передними/задними лапами
      float step_height = _parameters->Swing_traj_height + (_fin_foot_loc[foot][2] - pFoot[foot][2]) / 2.0;
      footSwingTrajectories[foot].setHeight(step_height);
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg =
        seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

      if (!data.userParameters->use_wbc)
      {
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;

        // singularity barrier
        data._legController->commands[foot].tauFeedForward[2] =
          50 * (data._legController->datas[foot].q(2) < .1) * data._legController->datas[foot].q(2);
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg =
        seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if (!data.userParameters->use_wbc)
      {
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint =
          Vec3<float>(_parameters->Kd_joint_0, _parameters->Kd_joint_1, _parameters->Kd_joint_2).asDiagonal();
      }
      se_contactState[foot] = contactState;
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.;
  pBody_RPY_des[2] = rpy_des[2];

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = v_rpy_des[2];

  // contact_state = gait->getContactState();
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

    if (current_gait == 4)
    {
      float trajInitial[12] = {
        (float)rpy_des[0], // Roll
        (float)rpy_des[1], // Pitch
        (float)stand_traj[5], (float)stand_traj[0], (float)stand_traj[1], (float)_body_height, 0, 0, 0, 0, 0, 0
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

      float trajInitial[12] = { (float)rpy_comp[0],  // 0
                                (float)rpy_comp[1],  // 1
                                (float)rpy_des[2],   // 2
                                xStart,              // 3
                                yStart,              // 4
                                (float)_body_height, // 5
                                0,                   // 6
                                0,                   // 7
                                (float)v_rpy_des[2], // 8
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
          trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * v_rpy_des[2];
        }
      }
    }
    solveDenseMPC(mpcTable, data);
  }
}

void VisionMPCLocomotion::solveDenseMPC(int* mpcTable, ControlFSMData<float>& data)
{
  auto seResult = data._stateEstimator->getResult();

  //  float Q[12] = { 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2 }; // prev
  float Q[12] = { 2.5, 2.5, 10, 50, 50, 100, 0, 0, 0.5, 0.2, 0.2, 0.1 };
  // float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for (int i = 0; i < 12; i++)
    r[i] = pFoot[i % 4][i / 4] - seResult.position[i / 4];

  if (alpha > 1e-4)
  {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  dtMPC = dt * iterationsBetweenMPC;
  vision_setup_problem(dtMPC, horizonLength, 0.4, 120);
  vision_update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);

  for (int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for (int axis = 0; axis < 3; axis++)
      f[axis] = vision_get_solution(leg * 3 + axis);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}
