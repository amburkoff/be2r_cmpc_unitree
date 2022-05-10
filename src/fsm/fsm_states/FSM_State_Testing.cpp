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

    pFoot[leg] = seResult.position + seResult.rBody.transpose() * (this->_data->_quadruped->getHipLocation(leg) + this->_data->_legController->datas[leg].p);
    firstSwing[leg] = true;
  }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Testing<T>::run()
{
  float duration = 1;
  float rate = 0.5;
  auto& seResult = this->_data->_stateEstimator->getResult();

  //x 0.047
  //y -0.15
  //z -0.073
  Vec3<float> pDes(0.047, -0.15, -0.073);
  static Vec3<float> pDes1(0.047, -0.15, -0.073);
  static Vec3<float> pDes0(_ini_foot_pos[0]);
  static bool flag = false;

  T progress = rate * iter * this->_data->controlParameters->controller_dt;

  if (progress > duration)
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
    // if (firstSwing[foot])
    // {
    //   firstSwing[foot] = false;
    //   footSwingTrajectories[foot].setHeight(0.1);
    //   footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
    //   footSwingTrajectories[foot].setFinalPosition(pFoot[foot] + Vec3<float>(1, 1, 0));
    // }

    // pFoot[foot] = seResult.position + seResult.rBody.transpose() * (this->_data->_quadruped->getHipLocation(foot) + this->_data->_legController->datas[foot].p);
    // footSwingTrajectories[foot].computeSwingTrajectoryBezier(progress, 5);

    // Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
    // Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
    // Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - this->_data->_quadruped->getHipLocation(foot);
    // Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

    // this->_data->_legController->commands[foot].pDes = pDesLeg;
    // this->_data->_legController->commands[foot].vDes = vDesLeg;
    // this->_data->_legController->commands[foot].kpCartesian = Vec3<T>(800, 800, 800).asDiagonal();
    // this->_data->_legController->commands[foot].kdCartesian = Vec3<T>(20, 20, 20).asDiagonal();

    if (flag == 0)
    {
      this->_data->_legController->commands[foot].pDes[0] = progress * (pDes1(0)) + (1. - progress) * pDes0(0);
      this->_data->_legController->commands[foot].pDes[1] = progress * (pDes1(1)) + (1. - progress) * pDes0(1);
      this->_data->_legController->commands[foot].pDes[2] = progress * (pDes1(2)) + (1. - progress) * pDes0(2);
    }
    else if (flag == 1)
    {
      this->_data->_legController->commands[foot].pDes[0] = progress * (pDes0(0)) + (1. - progress) * pDes1(0);
      this->_data->_legController->commands[foot].pDes[1] = progress * (pDes0(1)) + (1. - progress) * pDes1(1);
      this->_data->_legController->commands[foot].pDes[2] = progress * (pDes0(2)) + (1. - progress) * pDes1(2);
    }
  }
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
}

// template class FSM_State_Testing<double>;
template class FSM_State_Testing<float>;
