/*============================= Testing ==============================*/
/**
 * State for be2r testing cases
 */

#include "FSM_State_Testing.h"

using namespace std;

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Testing<T>::FSM_State_Testing(ControlFSMData<T>* _controlFSMData) : FSM_State<T>(_controlFSMData, FSM_StateName::TESTING, "TESTING"), _ini_foot_pos(4)
{
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_Testing<T>::onEnter()
{
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  auto& seResult = this->_data->_stateEstimator->getResult();

  // Reset iteration counter
  iter = 0;

  for (size_t leg(0); leg < 4; ++leg)
  {
    _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;

    // pFoot[leg] = seResult.position + seResult.rBody.transpose() * (this->_data->_quadruped->getHipLocation(leg) + this->_data->_legController->datas[leg].p);
    // pFoot[leg] = Vec3<float>(-0, -0.15, -0.2);

    firstSwing[leg] = true;
  }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Testing<T>::run()
{
  float rate = 1;
  float duration = 1 / rate;
  auto& seResult = this->_data->_stateEstimator->getResult();

  // Vec3<float> p0(0, 0, 0);
  // Vec3<float> p1(0, 0, 0);
  Vec3<float> p0(0, -0.15, -0.2);
  Vec3<float> p1(0, -0.25, -0.201);

  //near sholder
  //x 0.047
  //y -0.15
  //z -0.073

  //far 1
  //x 0.068
  //y -0.255
  //z -0.259

  //far 2
  //x -0.178
  //y -0.163
  //z -0.2
  Vec3<float> pDes(0.047, -0.15, -0.073);
  // static Vec3<float> pDes1(0.047, -0.15, -0.073);
  // static Vec3<float> pDes1(0.047, -0.15, -0.1);
  // static Vec3<float> pDes1(-0.178, -0.163, -0.2);
  // static Vec3<float> pDes0(_ini_foot_pos[0]);
  // static Vec3<float> pDes0(0.068, -0.255, -0.259);
  static bool flag = false;

  T progress = rate * iter * this->_data->controlParameters->controller_dt;

  auto _model = this->_data->_quadruped->buildModel();

  FBModelState<float> _state;
  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);

  _state.bodyOrientation = seResult.orientation;
  _state.bodyPosition = seResult.position;
  DVec<T> _full_config(cheetah::num_act_joint + 7);

  _full_config.setZero();

  for (size_t i(0); i < 3; ++i)
  {
    _state.bodyVelocity[i] = seResult.omegaBody[i];
    _state.bodyVelocity[i + 3] = seResult.vBody[i];

    for (size_t leg(0); leg < 4; ++leg)
    {
      _state.q[3 * leg + i] = this->_data->_legController->datas[leg].q[i];
      _state.qd[3 * leg + i] = this->_data->_legController->datas[leg].qd[i];

      _full_config[3 * leg + i + 6] = _state.q[3 * leg + i];
    }
  }

  _model.setState(_state);

  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  auto _A = _model.getMassMatrix();
  auto _grav = _model.getGravityForce();
  auto _coriolis = _model.getCoriolisForce();

  // cout << "grav: " << _grav << endl;
  Vec3<float> tau = _grav.segment(6, 3);
  // tau(0) = tau(0);
  // tau(1) = 0;
  // tau(2) = 0;
  // cout << "grav leg0: " << tau << endl;
  // cout << "c+g: " << _grav+_coriolis << endl;

  if (progress > 1)
  {
    // progress = duration;
    progress = 0;
    iter = 0;
    flag = !flag;
  }

  //for real
  // float p = 1200;
  // float d = 15;

  //for sim
  // float p = 800;
  // float d = 15;

  this->_data->_legController->setLegEnabled(0, true);
  this->_data->_legController->setLegEnabled(1, false);
  this->_data->_legController->setLegEnabled(2, false);
  this->_data->_legController->setLegEnabled(3, false);

  for (int foot = 0; foot < 4; foot++)
  {
    if (firstSwing[foot])
    {
      firstSwing[foot] = false;
      footSwingTrajectories[foot].setHeight(0.05);
      // footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      // footSwingTrajectories[foot].setFinalPosition(pFoot[foot] + Vec3<float>(1, 1, 0));
    }

    // this->_data->_legController->commands[foot].kpCartesian = Vec3<T>(800, 800, 800).asDiagonal();
    // this->_data->_legController->commands[foot].kdCartesian = Vec3<T>(20, 20, 20).asDiagonal();

    // this->_data->_legController->commands[foot].pDes = pDes;
    // this->_data->_legController->commands[foot].vDes = Vec3<float>::Constant(0);

    if (flag == 0)
    {
      footSwingTrajectories[foot].setInitialPosition(p0);
      footSwingTrajectories[foot].setFinalPosition(p1);

      //   this->_data->_legController->commands[foot].pDes[0] = progress * (pDes1(0)) + (1. - progress) * pDes0(0);
      //   this->_data->_legController->commands[foot].pDes[1] = progress * (pDes1(1)) + (1. - progress) * pDes0(1);
      //   this->_data->_legController->commands[foot].pDes[2] = progress * (pDes1(2)) + (1. - progress) * pDes0(2);
      this->_data->_legController->commands[foot].tauFeedForward = tau;
      //   Vec3<float> L = pDes1 - pDes0;
      //   this->_data->_legController->commands[foot].vDes = L / duration;
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(progress, 5);
    }
    else if (flag == 1)
    {
      footSwingTrajectories[foot].setInitialPosition(p1);
      footSwingTrajectories[foot].setFinalPosition(p0);
      footSwingTrajectories[foot].computeSwingTrajectoryModified(progress, 5, 1);
      //   this->_data->_legController->commands[foot].pDes[0] = progress * (pDes0(0)) + (1. - progress) * pDes1(0);
      //   this->_data->_legController->commands[foot].pDes[1] = progress * (pDes0(1)) + (1. - progress) * pDes1(1);
      //   this->_data->_legController->commands[foot].pDes[2] = progress * (pDes0(2)) + (1. - progress) * pDes1(2);
        this->_data->_legController->commands[foot].tauFeedForward = tau;
      //   Vec3<float> L = pDes0 - pDes1;
      //   this->_data->_legController->commands[foot].vDes = L / duration;
    }

    

    Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
    Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
    // Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - this->_data->_quadruped->getHipLocation(foot);
    // Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

    // this->_data->_legController->commands[foot].pDes = pDesLeg;
    // this->_data->_legController->commands[foot].vDes = vDesLeg;
    // Kp = {150, 0, 0, 0, 150, 0, 0, 0, 150};
    // Kp_stance = Kp;

    // Kd = {3, 0, 0, 0, 3, 0, 0, 0, 3};
    // Kd_stance = Kd;
    this->_data->_legController->commands[foot].pDes = pDesFootWorld;
    this->_data->_legController->commands[foot].vDes = vDesFootWorld;
    // data._legController->commands[foot].kpCartesian = Kp;
    // data._legController->commands[foot].kdCartesian = Kd;
  }

  Vec3<float> p_des = footSwingTrajectories[0].getPosition();
  Vec3<float> v_des = footSwingTrajectories[0].getVelocity();

  // static Vec3<float> q_des(0, 0, 0);
  // static Vec3<float> dq_des(0, 0, 0);
  // q_des(1) = -0.5 * sin((float)iter / 1000.0) - 0.5 - 0.5;
  // q_des(2) = 0.5 * sin((float)iter / 1000.0) + 0.5 + 1.5;

  Vec3<float> q_des(0, 0, 0);
  Vec3<float> dq_des(0, 0, 0);

  q_des = this->findAngles(0, p_des);
  //dq_des = computeLegJacobianAndPosition
  // this->jointPDControl(0, q_des, dq_des);
  this->lowLeveljointPDControl(0, q_des, dq_des);
  // this->_data->_legController->is_low_level = true;

  // Vec3<float> p_act = this->_data->_legController->datas[0].p;
  // Vec3<float> q_eval = this->findAngles(0, p_act);
  // cout << "q_eval: " << q_eval << endl;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Testing<T>::checkTransition()
{
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode)
  {
  case K_TESTING:
    break;

  case K_STAND_UP:
    // Requested switch to Stand Up
    this->nextStateName = FSM_StateName::STAND_UP;
    break;

  case K_PASSIVE: // normal c
    this->nextStateName = FSM_StateName::PASSIVE;
    break;

  default:
    std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
              << K_TESTING << " to "
              << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Testing<T>::transition()
{
  // Finish Transition
  switch (this->nextStateName)
  {
  case FSM_StateName::PASSIVE: // normal
    this->transitionData.done = true;
    break;

  case FSM_StateName::STAND_UP:
    this->transitionData.done = true;
    break;

  default:
    std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Testing<T>::onExit()
{
  // Nothing to clean up when exiting
  // this->_data->_legController->zeroCommand();

  this->_data->_legController->setEnabled(false);
  // this->_data->_legController->is_low_level = true;
}

// template class FSM_State_Testing<double>;
template class FSM_State_Testing<float>;
