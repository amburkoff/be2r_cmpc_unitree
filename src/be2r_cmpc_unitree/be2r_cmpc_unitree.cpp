#include "be2r_cmpc_unitree.hpp"

using namespace std;

Body_Manager::Body_Manager(RobotController* robot_ctrl)
{
  _robot_ctrl = robot_ctrl;
  _userControlParameters = robot_ctrl->getUserControlParameters();
}

Body_Manager::~Body_Manager()
{
  delete _legController;
  delete _stateEstimator;
}

void Body_Manager::init()
{
  _initSubscribers();
  _initPublishers();

  printf("[Hardware Bridge] Loading parameters from file...\n");

  try
  {
    controlParameters.initializeFromYamlFile(THIS_COM "config/mini-cheetah-defaults.yaml");
  }
  catch (std::exception& e)
  {
    printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
    exit(1);
  }

  if (!controlParameters.isFullyInitialized())
  {
    printf("Failed to initialize all robot parameters\n");
    exit(1);
  }

  printf("Loaded robot parameters\n");

  if (_userControlParameters)
  {
    try
    {
      _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters.yaml");
    }
    catch (std::exception& e)
    {
      printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
      exit(1);
    }

    if (!_userControlParameters->isFullyInitialized())
    {
      printf("Failed to initialize all user parameters\n");
      exit(1);
    }

    printf("Loaded user parameters\n");
  }
  else
  {
    printf("Did not load user parameters because there aren't any\n");
  }

  _quadruped = buildMiniCheetah<float>();

  // Initialize the model and robot data
  _model = _quadruped.buildModel();
  // _jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt); // HERE mb delete this?

  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(&vectorNavData, _legController->datas, &_stateEstimate, &controlParameters);
  initializeStateEstimator();

  // Initialize the DesiredStateCommand object
  _desiredStateCommand = new DesiredStateCommand<float>(&driverCommand, &controlParameters, &_stateEstimate, controlParameters.controller_dt);

  // Controller initializations
  _robot_ctrl->_model = &_model;
  _robot_ctrl->_quadruped = &_quadruped;
  _robot_ctrl->_legController = _legController;
  _robot_ctrl->_stateEstimator = _stateEstimator;
  _robot_ctrl->_stateEstimate = &_stateEstimate;
  // _robot_ctrl->_visualizationData = visualizationData;
  _robot_ctrl->_robotType = RobotType::MINI_CHEETAH;
  _robot_ctrl->_driverCommand = &driverCommand;
  _robot_ctrl->_controlParameters = &controlParameters;
  _robot_ctrl->_desiredStateCommand = _desiredStateCommand;

  _robot_ctrl->initializeController();
}

void Body_Manager::run()
{
  // Run the state estimator step
  _stateEstimator->run();

  // Update the data from the robot
  setupStep();

  static int count_ini(0);
  ++count_ini;

  if (count_ini < 10)
  {
    _legController->setEnabled(false);
  }
  else
  {
    _legController->setEnabled(true);

    // Run Control
    _robot_ctrl->runController();
  }

  // Sets the leg controller commands for the robot appropriate commands
  finalizeStep();

  if (count_ini > 100 && count_ini < 1500)
  {
    ROS_INFO_STREAM_ONCE("Stand up " << count_ini);

    controlParameters.control_mode = 1;
  }
  else if (count_ini > 1500)
  {
    ROS_INFO_STREAM_ONCE("Locomotion " << count_ini);

    controlParameters.control_mode = FSM;
  }
}

void Body_Manager::setupStep()
{
  _legController->updateData(&spiData);

  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5);

  // todo safety checks, sanity checks, etc...
}

void Body_Manager::finalizeStep()
{

  _legController->updateCommand(&spiCommand);

  _iterations++;

  for (int i = 0; i < 4; i++)
  {
    _torqueCalculator(&spiCommand, &spiData, &_spi_torque, i);
  }

  static unitree_legged_msgs::LowCmd _low_cmd;

  for (uint8_t leg_num = 0; leg_num < 4; leg_num++)
  {
    _low_cmd.motorCmd[leg_num * 3 + 0].tau = _spi_torque.tau_abad[leg_num];
    _low_cmd.motorCmd[leg_num * 3 + 1].tau = -_spi_torque.tau_hip[leg_num];
    _low_cmd.motorCmd[leg_num * 3 + 2].tau = -_spi_torque.tau_knee[leg_num];
  }

  _pub_low_cmd.publish(_low_cmd);
}

/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void Body_Manager::initializeStateEstimator()
{
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);

  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
}

void Body_Manager::_initSubscribers()
{
  _sub_low_state = _nh.subscribe("/low_state", 1, &Body_Manager::_lowStateCallback, this, ros::TransportHints().tcpNoDelay(true));
  _sub_cmd_vel = _nh.subscribe("/cmd_vel", 1, &Body_Manager::_cmdVelCallback, this, ros::TransportHints().tcpNoDelay(true));
}

void Body_Manager::_initPublishers()
{
  _pub_low_cmd = _nh.advertise<unitree_legged_msgs::LowCmd>("/low_cmd", 1);
}

void Body_Manager::_lowStateCallback(unitree_legged_msgs::LowState msg)
{
  // ROS_INFO("lsc");

  for (uint8_t leg_num = 0; leg_num < 4; leg_num++)
  {
    spiData.q_abad[leg_num] = msg.motorState[leg_num * 3 + 0].q;
    spiData.q_hip[leg_num] = -msg.motorState[leg_num * 3 + 1].q;
    spiData.q_knee[leg_num] = -msg.motorState[leg_num * 3 + 2].q;

    spiData.qd_abad[leg_num] = msg.motorState[leg_num * 3 + 0].dq;
    spiData.qd_hip[leg_num] = -msg.motorState[leg_num * 3 + 1].dq;
    spiData.qd_knee[leg_num] = -msg.motorState[leg_num * 3 + 2].dq;
  }

  is_stand = msg.levelFlag;

  vectorNavData.gyro[0] = msg.imu.gyroscope[0];
  vectorNavData.gyro[1] = msg.imu.gyroscope[1];
  vectorNavData.gyro[2] = msg.imu.gyroscope[2];

  vectorNavData.accelerometer[0] = msg.imu.accelerometer[0];
  vectorNavData.accelerometer[1] = msg.imu.accelerometer[1];
  vectorNavData.accelerometer[2] = msg.imu.accelerometer[2];

  // vectorNavData.quat[0] = msg.imu.quaternion[0]; //w
  // vectorNavData.quat[1] = msg.imu.quaternion[1]; //x
  // vectorNavData.quat[2] = msg.imu.quaternion[2]; //y
  // vectorNavData.quat[3] = msg.imu.quaternion[3]; //z

  vectorNavData.quat[0] = msg.imu.quaternion[1]; //w
  vectorNavData.quat[1] = msg.imu.quaternion[2]; //x
  vectorNavData.quat[2] = msg.imu.quaternion[3]; //y
  vectorNavData.quat[3] = msg.imu.quaternion[0]; //z
}

void Body_Manager::_cmdVelCallback(geometry_msgs::Twist msg)
{
  driverCommand.leftStickAnalog[1] = msg.linear.x;
  driverCommand.leftStickAnalog[0] = -msg.linear.y;

  driverCommand.rightStickAnalog[0] = -msg.angular.z;
}

/*!
 * Emulate the spi board to estimate the torque.
 */
void Body_Manager::_torqueCalculator(SpiCommand* cmd, SpiData* data, spi_torque_t* torque_out, int board_num)
{
  torque_out->tau_abad[board_num] = cmd->kp_abad[board_num] * (cmd->q_des_abad[board_num] - data->q_abad[board_num]) +
                                    cmd->kd_abad[board_num] * (cmd->qd_des_abad[board_num] - data->qd_abad[board_num]) +
                                    cmd->tau_abad_ff[board_num];

  torque_out->tau_hip[board_num] = cmd->kp_hip[board_num] * (cmd->q_des_hip[board_num] - data->q_hip[board_num]) +
                                   cmd->kd_hip[board_num] * (cmd->qd_des_hip[board_num] - data->qd_hip[board_num]) +
                                   cmd->tau_hip_ff[board_num];

  torque_out->tau_knee[board_num] = cmd->kp_knee[board_num] * (cmd->q_des_knee[board_num] - data->q_knee[board_num]) +
                                    cmd->kd_knee[board_num] * (cmd->qd_des_knee[board_num] - data->qd_knee[board_num]) +
                                    cmd->tau_knee_ff[board_num];

  // const float* torque_limits = disabled_torque;
  const float* torque_limits = max_torque;
  // const float* torque_limits = max_max_torque;

  // if (cmd->flags[board_num] & 0b1)
  // {
  //   if (cmd->flags[board_num] & 0b10)
  //     torque_limits = wimp_torque;
  //   else
  //     torque_limits = max_torque;
  // }

  if (torque_out->tau_abad[board_num] > torque_limits[0])
  {
    torque_out->tau_abad[board_num] = torque_limits[0];
  }
  if (torque_out->tau_abad[board_num] < -torque_limits[0])
  {
    torque_out->tau_abad[board_num] = -torque_limits[0];
  }
  if (torque_out->tau_hip[board_num] > torque_limits[1])
  {
    torque_out->tau_hip[board_num] = torque_limits[1];
  }
  if (torque_out->tau_hip[board_num] < -torque_limits[1])
  {
    torque_out->tau_hip[board_num] = -torque_limits[1];
  }
  if (torque_out->tau_knee[board_num] > torque_limits[2])
  {
    torque_out->tau_knee[board_num] = torque_limits[2];
  }
  if (torque_out->tau_knee[board_num] < -torque_limits[2])
  {
    torque_out->tau_knee[board_num] = -torque_limits[2];
  }

  // float q0_e = cmd->q_des_abad[board_num] - data->q_abad[board_num];
  // float q1_e = cmd->q_des_hip[board_num] - data->q_hip[board_num];
  // float q2_e = cmd->q_des_knee[board_num] - data->q_knee[board_num];

  // cout << "Leg: " << board_num << " q0_e: " << q0_e << " q1_e: " << q1_e << " q2_e: " << q2_e << endl;
  // cout << "Leg: " << board_num << " t0: " << torque_out->tau_abad[board_num] << " t1: " << torque_out->tau_hip[board_num] << " t2: " << torque_out->tau_knee[board_num] << endl;
}
