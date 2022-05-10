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
template<typename T>
FSM_State_Testing<T>::FSM_State_Testing(ControlFSMData<T>* _controlFSMData)
  : FSM_State<T>(_controlFSMData, FSM_StateName::TESTING, "TESTING")
  , _ini_foot_pos(4)
{
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template<typename T>
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

    pFoot[leg] = seResult.position +
                 seResult.rBody.transpose() * (this->_data->_quadruped->getHipLocation(leg) +
                                               this->_data->_legController->datas[leg].p);
    firstSwing[leg] = true;
  }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_Testing<T>::run()
{

  float duration = 1;
  float rate = 0.25;
  auto& seResult = this->_data->_stateEstimator->getResult();
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;

  // x 0.047
  // y -0.15
  // z -0.073

  Vec3<float> pDes(0.047, -0.15, -0.073);

  T progress = rate * iter * this->_data->controlParameters->controller_dt;

  this->_data->_legController->setEnabled(true);

  if (progress > duration)
  {
    progress = duration;
  }

  // for real
  // float p = 1200;
  // float d = 15;
  Kp << 500, 0, 0, 0, 500, 0, 0, 0, 400;
  Kp_stance = 0 * Kp;

  Kd << 2, 0, 0, 0, 2, 0, 0, 0, 2;
  Kd_stance = Kd;

  // for sim
  // float p = 800;
  // float d = 15;

  this->_data->_legController->setLegEnabled(0, true);
  this->_data->_legController->setLegEnabled(1, false);
  this->_data->_legController->setLegEnabled(2, false);
  this->_data->_legController->setLegEnabled(3, false);

  for (int foot = 0; foot < 4; foot++)
  {
    Vec3<float> qDes(-0.1469, -0.8319, 1.4418);
    Vec3<float> qdDes(0, 0, 0);
    this->jointPDControl(foot, qDes, qdDes);
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template<typename T>
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
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << K_TESTING << " to "
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
template<typename T>
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
template<typename T>
void FSM_State_Testing<T>::onExit()
{
  // Nothing to clean up when exiting
}

// template class FSM_State_Testing<double>;
template class FSM_State_Testing<float>;
