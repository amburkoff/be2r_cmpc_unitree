/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_StandUp.h"

using namespace std;

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T>* _controlFSMData) : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
                                                                              _ini_foot_pos(4)
{
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;

  // f = boost::bind(&callbackROSros, _1, _2);
  // server.setCallback(f);

  // ROS_INFO("START SERVER");
}

template <typename T>
void FSM_State_StandUp<T>::onEnter()
{
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter = 0;

  for (size_t leg(0); leg < 4; ++leg)
  {
    _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
  }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_StandUp<T>::run()
{
  T hMax = 0.25;
  T progress = 0.5 * iter * this->_data->controlParameters->controller_dt;

  if (progress > 1.)
  {
    progress = 1.;
  }

  FloatingBaseModel<float> _model = this->_data->_legController->_quadruped.buildModel();

  const StateEstimate<float>& state_est = this->_data->_stateEstimator->getResult();
  const LegControllerData<float>* leg_data = this->_data->_legController->datas;

  FBModelState<float> _state;
  DVec<float> _full_config(cheetah::num_act_joint + 7);

  _state.q = DVec<float>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<float>::Zero(cheetah::num_act_joint);
  _full_config.setZero();

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

      _full_config[3 * leg + i + 6] = _state.q[3 * leg + i];
    }
  }
  _model.setState(_state);

  _model.generalizedGravityForce();

  Vec12<float> gravity = _model.getGravityForce().tail(12);

  // cout << "gravity: " << gravity << endl;

  for (int i = 0; i < 4; i++)
  {
    // this->_data->_legController->commands[i].kpCartesian = Vec3<T>(600, 600, 600).asDiagonal();
    this->_data->_legController->commands[i].kpCartesian = Vec3<T>(1000, 1000, 1000).asDiagonal();
    this->_data->_legController->commands[i].kdCartesian = Vec3<T>(10, 10, 10).asDiagonal();

    this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
    this->_data->_legController->commands[i].pDes[2] = progress * (-hMax) + (1. - progress) * _ini_foot_pos[i][2];

    this->_data->_legController->commands[i].tauFeedForward = gravity.block<3,1>(i * 3, 0);

    // cout << "tau ff " << i << ": " << this->_data->_legController->commands[i].tauFeedForward << endl;

    // cout << this->_data->_legController->commands[i].pDes << endl;
    // cmd = tot_tau.tail(WB::num_act_joint_);

    // this->_data->_legController->commands[i].qDes(0) = _ini_foot_pos[i];
    // this->_data->_legController->commands[i].qDes(0) = _ini_foot_pos[i];
    // this->_data->_legController->commands[i].qDes(0) = _ini_foot_pos[i];
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_StandUp<T>::checkTransition()
{
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode)
  {
  case K_STAND_UP:
    break;

  case K_BALANCE_STAND:
    this->nextStateName = FSM_StateName::BALANCE_STAND;
    break;

  case K_LOCOMOTION:
    this->nextStateName = FSM_StateName::LOCOMOTION;
    break;

  case K_VISION:
    this->nextStateName = FSM_StateName::VISION;
    break;

  case K_PASSIVE: // normal c
    this->nextStateName = FSM_StateName::PASSIVE;
    break;

  case K_LAY_DOWN: // normal c
    ROS_INFO("CHeck to lay down");
    this->nextStateName = FSM_StateName::LAYDOWN;
    break;

  default:
    std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
              << K_PASSIVE << " to "
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
TransitionData<T> FSM_State_StandUp<T>::transition()
{
  // Finish Transition
  switch (this->nextStateName)
  {
  case FSM_StateName::PASSIVE: // normal
    this->transitionData.done = true;
    break;

  case FSM_StateName::BALANCE_STAND:
    this->transitionData.done = true;
    break;

  case FSM_StateName::LOCOMOTION:
    this->transitionData.done = true;
    break;

  case FSM_StateName::VISION:
    this->transitionData.done = true;
    break;

  case FSM_StateName::LAYDOWN:
    this->transitionData.done = true;
    break;

  default:
    std::cout << "[CONTROL FSM] Something went wrong in transition"
              << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_StandUp<T>::onExit()
{
  // Nothing to clean up when exiting
}

// template class FSM_State_StandUp<double>;
template class FSM_State_StandUp<float>;
