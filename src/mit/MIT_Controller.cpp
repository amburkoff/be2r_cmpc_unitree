#include "MIT_Controller.hpp"

using namespace std;

MIT_Controller::MIT_Controller() : RobotController()
{
}

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void MIT_Controller::initializeController()
{
  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);

  // Initialize a new ContactEstimator object
  //_contactEstimator = new ContactEstimator<double>();
  //_contactEstimator->initialize();

  // Initializes the Control FSM with all the required data
  _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
                                      _legController, _gaitScheduler,
                                      _desiredStateCommand,
                                      _controlParameters,
                                      // _visualizationData,
                                      &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void MIT_Controller::runController()
{
  // cout << "[MIT_Controller] Start ctrl loop" << endl;

  // Find the current gait schedule
  _gaitScheduler->step();

  // cout << "[MIT_Controller] GS step done" << endl;

  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands();

  // cout << "[MIT_Controller] DS convert done" << endl;

  // Run the Control FSM code
  _controlFSM->runFSM();

  // cout << "[MIT_Controller] FSM done" << endl;
}
