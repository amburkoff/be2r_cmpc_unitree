/*============================= Stairs ==============================*/
/**
 * State for be2r Stairs cases
 */

#include "FSM_State_Stairs.h"
#include <Utilities/Timer.h>
#include <controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
// #include <rt/rt_interface_lcm.h>

//оригинальный параметр для MPC+WBC
#define ITERATIONS_BETWEEN_MPC 13
// #define ITERATIONS_BETWEEN_MPC 26 //1000 Hz

//лучший для только MPC
// #define ITERATIONS_BETWEEN_MPC 10
using namespace std;

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Stairs<T>::FSM_State_Stairs(ControlFSMData<T>* _controlFSMData) : FSM_State<T>(_controlFSMData, FSM_StateName::STAIRS, "STAIRS")
{
  cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt, ITERATIONS_BETWEEN_MPC, _controlFSMData->userParameters);
  // cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
  //                                   //30 / (1000. * _controlFSMData->controlParameters->controller_dt),
  //                                   //22 / (1000. * _controlFSMData->controlParameters->controller_dt),
  //                                   27 / (1000. * _controlFSMData->controlParameters->controller_dt),
  //                                   _controlFSMData->userParameters);

  this->turnOnAllSafetyChecks();
  // this->turnOffAllSafetyChecks();

  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF and footstep locations to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();
  this->footstepLocations = Mat34<T>::Zero();
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();
}

template <typename T>
void FSM_State_Stairs<T>::onEnter()
{
  cout << "[FSM_State_Stairs] onEnter start" << endl;

  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();
  cout << "[FSM_State_Stairs] transitionData zero done" << endl;

  cMPCOld->initialize();
  cout << "[FSM_State_Stairs] cMPCOld initialize done" << endl;

  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
  printf("[FSM LOCOMOTION] On Enter\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Stairs<T>::run()
{
  // Call the locomotion control logic for this iteration
  LocomotionControlStep();
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Stairs<T>::checkTransition()
{
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode)
  {
  case K_STAIRS:
    break;


  case K_BALANCE_STAND:
      // Requested change to BALANCE_STAND
      this->nextStateName = FSM_StateName::BALANCE_STAND;

      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

    case K_PASSIVE:
      // Requested change to BALANCE_STAND
      this->nextStateName = FSM_StateName::PASSIVE;

      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

    case K_STAND_UP:
      this->nextStateName = FSM_StateName::STAND_UP;
      this->transitionDuration = 0.;
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      this->transitionDuration = 0.;
      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      this->transitionDuration = 0.;
      break;

    case K_LAY_DOWN:
      this->nextStateName = FSM_StateName::LAYDOWN;
      this->transitionDuration = 0.;
      break;

  default:
    std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
              << K_STAIRS << " to "
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
TransitionData<T> FSM_State_Stairs<T>::transition()
{
  // Switch FSM control mode
  switch (this->nextStateName)
  {
  case FSM_StateName::BALANCE_STAND:
    LocomotionControlStep();

    iter++;
    if (iter >= this->transitionDuration * 1000)
    {
      this->transitionData.done = true;
    }
    else
    {
      this->transitionData.done = false;
    }

    break;

  case FSM_StateName::PASSIVE:
    this->turnOffAllSafetyChecks();

    this->transitionData.done = true;

    break;

  case FSM_StateName::STAND_UP:
    this->transitionData.done = true;
    break;

  case FSM_StateName::RECOVERY_STAND:
    this->transitionData.done = true;
    break;

  case FSM_StateName::LAYDOWN:
    this->transitionData.done = true;
    break;

  default:
    std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

template <typename T>
bool FSM_State_Stairs<T>::locomotionSafe()
{
  auto& seResult = this->_data->_stateEstimator->getResult();

  const T max_roll = 40;
  const T max_pitch = 40;

  if (std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll))
  {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if (std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch))
  {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for (int leg = 0; leg < 4; leg++)
  {
    auto p_leg = this->_data->_legController->datas[leg].p;
    if (p_leg[2] > 0)
    {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    if (std::fabs(p_leg[1] > 0.18))
    {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return false;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();
    if (std::fabs(v_leg) > 9.)
    {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }

  return true;
}
/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Stairs<T>::onExit()
{
  // Nothing to clean up when exiting
  // this->_data->_legController->zeroCommand();

  this->_data->_legController->setEnabled(false);
  this->_data->_legController->commands[0].is_low_level = true;
  this->_data->_legController->commands[1].is_low_level = true;
  this->_data->_legController->commands[2].is_low_level = true;
  this->_data->_legController->commands[3].is_low_level = true;
}

/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_Stairs<T>::LocomotionControlStep()
{
  // StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

  // Contact state logic
  // estimateContact();

  // cout << "[FSM_State_Stairs] LocomotionControlStep start" << endl;
  // this->_data->userParameters->use_wbc = 0;

  cMPCOld->run<T>(*this->_data);

  // cout << "[FSM_State_Stairs] cMPCOld done" << endl;

  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

  for (int leg(0); leg < 4; ++leg)
  {
    pDes_backup[leg] = this->_data->_legController->commands[leg].pDes;
    vDes_backup[leg] = this->_data->_legController->commands[leg].vDes;
    Kp_backup[leg] = this->_data->_legController->commands[leg].kpCartesian;
    Kd_backup[leg] = this->_data->_legController->commands[leg].kdCartesian;
  }

  if (this->_data->userParameters->use_wbc > 0.9)
  {
    _wbc_data->pBody_des = cMPCOld->pBody_des;
    _wbc_data->vBody_des = cMPCOld->vBody_des;
    _wbc_data->aBody_des = cMPCOld->aBody_des;

    _wbc_data->pBody_RPY_des = cMPCOld->pBody_RPY_des;
    _wbc_data->vBody_Ori_des = cMPCOld->vBody_Ori_des;

    for (size_t i(0); i < 4; ++i)
    {
      _wbc_data->pFoot_des[i] = cMPCOld->pFoot_des[i];
      _wbc_data->vFoot_des[i] = cMPCOld->vFoot_des[i];
      _wbc_data->aFoot_des[i] = cMPCOld->aFoot_des[i];
      _wbc_data->Fr_des[i] = cMPCOld->Fr_des[i];
    }
    _wbc_data->contact_state = cMPCOld->contact_state;
    _wbc_ctrl->run(_wbc_data, *this->_data);
  }

  for (int leg(0); leg < 4; ++leg)
  {
    //originally commented
    this->_data->_legController->commands[leg].pDes = pDes_backup[leg];
    this->_data->_legController->commands[leg].vDes = vDes_backup[leg];

    this->_data->_legController->commands[leg].kpCartesian = Kp_backup[leg];
    this->_data->_legController->commands[leg].kdCartesian = Kd_backup[leg];
  }
}

/**
 * Stance leg logic for impedance control. Prevent leg slipping and
 * bouncing, as well as tracking the foot velocity during high speeds.
 */
template <typename T>
void FSM_State_Stairs<T>::StanceLegImpedanceControl(int leg)
{
  // Impedance control for the stance leg
  this->cartesianImpedanceControl(leg, this->footstepLocations.col(leg), Vec3<T>::Zero(), this->_data->controlParameters->stand_kp_cartesian, this->_data->controlParameters->stand_kd_cartesian);
}

// template class FSM_State_Stairs<double>;
template class FSM_State_Stairs<float>;
