#ifndef FSM_STATE_RECOVERY_STANDUP_H
#define FSM_STATE_RECOVERY_STANDUP_H

#include "FSM_State.h"

enum class TypeOfMotion {
  kFoldLegs,
  kStandUp,
  kRollOver,
};

template <typename T>
class FSM_State_RecoveryStand : public FSM_State<T> {
 public:
  FSM_State_RecoveryStand(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  TransitionData<T> testTransition();

 private:
  // Keep track of the control iterations
  int iter_{};
  int motion_start_iter_{};

  unsigned long long state_iter_{};
  TypeOfMotion motion_flag_{TypeOfMotion::kFoldLegs};

  // JPos
  Vec3<T> fold_jpos_[4];
  Vec3<T> stand_jpos_[4];
  Vec3<T> rolling_jpos_[4];
  Vec3<T> initial_jpos_[4];
  Vec3<T> zero_vec3_{Vec3<T>::Zero()};

  Vec3<T> f_ff_{};

  const int kRolloverRampIter_{150};
  const int kRolloverSettleIter_{150};

  const int kFoldRampIter_{400};
  const int kFoldSettleIter_{700};

  const int kStandupRampIter_{500};
  const int kStandupSettleIter_{250};

  void RollOver_(int iter);
  void StandUp_(int iter);
  void FoldLegs_(int iter);

  bool IsUpsideDown_();
  void SetJPosInterPts_(size_t curr_iter, size_t max_iter, int leg, const Vec3<T>& ini,
                        const Vec3<T>& fin);
};

#endif  // FSM_STATE_RECOVERY_STANDUP_H
