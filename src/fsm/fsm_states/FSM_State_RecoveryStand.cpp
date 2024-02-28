/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_RecoveryStand.h"

#include <Utilities/Utilities_print.h>

#include "ros_read_param.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_RecoveryStand<T>::FSM_State_RecoveryStand(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP") {
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;

  /*
  // I don't know where but there is something wrong with signs.
  // Numbers with "-" are actually positive on joints
  // and numbers without "-" are negative on joints
  */

  // Folding joints configuration
  for (size_t leg{0}; leg < 4; ++leg) {
    fold_jpos_[leg] << 0.0f, -1.4f, 2.7f;
  }

  // Stanting joints configuration
  for (size_t leg{0}; leg < 4; ++leg) {
    stand_jpos_[leg] << 0.0f, -0.85f, 1.65f;
  }

  // Rolling joints configuration
  rolling_jpos_[0] << 1.5f, -1.6f, 2.77f;
  rolling_jpos_[1] << 1.3f, -3.5f, 1.0f;
  rolling_jpos_[2] << 1.5f, -1.6f, 2.77f;
  rolling_jpos_[3] << 1.3f, -3.5f, 1.0f;

  f_ff_ << 0.f, 0.f, -25.f;
}

template <typename T>
void FSM_State_RecoveryStand<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter_ = 0;
  state_iter_ = 0;

  // initial configuration, position
  for (size_t i(0); i < 4; ++i) {
    initial_jpos_[i] = this->_data->legController->datas[i].q;
  }

  T body_height = this->_data->stateEstimator->getResult().position[2];

  motion_flag_ = TypeOfMotion::kFoldLegs;

  if (!IsUpsideDown_()) {  // Proper orientation
    if ((0.1 < body_height) && (body_height < 0.45)) {
      printf("[Recovery Balance] body height is %f; Stand Up \n", body_height);
      motion_flag_ = TypeOfMotion::kStandUp;
    } else {
      printf("[Recovery Balance] body height is %f; Folding legs \n", body_height);
    }
  } else {
    printf("[Recovery Balance] UpsideDown (%d) \n", IsUpsideDown_());
  }

  motion_start_iter_ = 0;
}

template <typename T>
bool FSM_State_RecoveryStand<T>::IsUpsideDown_() {
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << '\n';
  std::cout << "Rot Matrix: " << this->_data->stateEstimator->getResult().rBody << '\n';
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << '\n';
  if (this->_data->stateEstimator->getResult().rBody(2, 2) < 0) {
    return true;
  }
  return false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_RecoveryStand<T>::run() {
  switch (motion_flag_) {
    case TypeOfMotion::kStandUp:
      StandUp_(state_iter_ - motion_start_iter_);
      break;
    case TypeOfMotion::kFoldLegs:
      FoldLegs_(state_iter_ - motion_start_iter_);
      break;
    case TypeOfMotion::kRollOver:
      RollOver_(state_iter_ - motion_start_iter_);
      break;
  }

  state_iter_++;
}

template <typename T>
void FSM_State_RecoveryStand<T>::SetJPosInterPts_(size_t current_iter, size_t max_iter, int leg,
                                                  const Vec3<T>& initial_configuration,
                                                  const Vec3<T>& final_configuration) {
  T a{0};
  T b{1};

  // if we're done interpolating
  if (current_iter <= max_iter) {
    b = static_cast<T>(current_iter) / static_cast<T>(max_iter);
    a = static_cast<T>(1) - b;
  }
  // compute setpoints
  Vec3<T> inter_pos = a * initial_configuration + b * final_configuration;
  // do control
  this->jointPDControl(leg, inter_pos, zero_vec3_);
}

template <typename T>
void FSM_State_RecoveryStand<T>::RollOver_(int curr_iter) {
  for (size_t i(0); i < 4; ++i) {
    SetJPosInterPts_(curr_iter, kRolloverRampIter_, i, initial_jpos_[i], rolling_jpos_[i]);
  }

  if (curr_iter > kRolloverRampIter_ + kRolloverSettleIter_) {
    motion_flag_ = TypeOfMotion::kFoldLegs;
    for (size_t i(0); i < 4; ++i) initial_jpos_[i] = rolling_jpos_[i];
    motion_start_iter_ = state_iter_ + 1;
  }
}

template <typename T>
void FSM_State_RecoveryStand<T>::StandUp_(int curr_iter) {
  // T body_height = this->_data->stateEstimator->getResult().position[2];

  // FIXME!
  // std::cout << "Body height: " << this->_data->stateEstimator->getResult().position[2] << '\n';

  for (size_t leg(0); leg < 4; ++leg) {
    SetJPosInterPts_(curr_iter, kStandupRampIter_, leg, initial_jpos_[leg], stand_jpos_[leg]);
  }

  // feed forward mass of robot.
  // for(int i = 0; i < 4; i++)
  // this->_data->legController->commands[i].forceFeedForward = f_ff_;
  // Vec4<T> se_contactState(0.,0.,0.,0.);
  Vec4<T> se_contactState(0.5, 0.5, 0.5, 0.5);
  this->_data->stateEstimator->setContactPhase(se_contactState);
}

template <typename T>
void FSM_State_RecoveryStand<T>::FoldLegs_(int curr_iter) {
  for (size_t i(0); i < 4; ++i) {
    SetJPosInterPts_(curr_iter, kFoldRampIter_, i, initial_jpos_[i], fold_jpos_[i]);
  }
  if (curr_iter >= kFoldRampIter_ + kFoldSettleIter_) {
    if (IsUpsideDown_()) {
      motion_flag_ = TypeOfMotion::kRollOver;
      for (size_t i(0); i < 4; ++i) initial_jpos_[i] = fold_jpos_[i];
    } else {
      motion_flag_ = TypeOfMotion::kStandUp;
      for (size_t i(0); i < 4; ++i) initial_jpos_[i] = fold_jpos_[i];
    }
    motion_start_iter_ = state_iter_ + 1;
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_RecoveryStand<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter_++;

  // Switch FSM control mode
  switch ((int)this->_data->userParameters->FSM_State) {
    case K_RECOVERY_STAND:
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;

    case K_PASSIVE:
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    case K_BACKFLIP:
      this->nextStateName = FSM_StateName::BACKFLIP;
      break;

    case K_FRONTJUMP:
      this->nextStateName = FSM_StateName::FRONTJUMP;
      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << K_RECOVERY_STAND
                << " to " << this->_data->userParameters->FSM_State << std::endl;
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
TransitionData<T> FSM_State_RecoveryStand<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::BACKFLIP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::FRONTJUMP:
      this->transitionData.done = true;
      break;
      // Den is a loh-doh

    case FSM_StateName::VISION:
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
void FSM_State_RecoveryStand<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_RecoveryStand<double>;
template class FSM_State_RecoveryStand<float>;
