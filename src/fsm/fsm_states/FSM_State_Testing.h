#ifndef FSM_STATE_TESTING_H
#define FSM_STATE_TESTING_H

#include "FSM_State.h"
#include <FootSwingTrajectory.h>
#include <LegController.h>

/**
 *
 */
template <typename T>
class FSM_State_Testing : public FSM_State<T>
{
public:
  FSM_State_Testing(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();
  
  void test1();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  TransitionData<T> testTransition();

private:
  // Keep track of the control iterations
  int iter = 0;
  std::vector<Vec3<T>> _ini_foot_pos;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  bool firstSwing[4];
  Vec3<float> pFoot[4];
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
};

#endif // FSM_STATE_TESTING_H
