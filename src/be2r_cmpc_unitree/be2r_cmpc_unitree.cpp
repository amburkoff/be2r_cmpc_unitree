#include "be2r_cmpc_unitree.hpp"

using namespace std;

Body_Manager::Body_Manager()
    : _zero_time(0), safe(UNITREE_LEGGED_SDK::LeggedType::A1), udp(UNITREE_LEGGED_SDK::LOWLEVEL)
{
  footContactState = Vec4<uint8_t>::Zero();
  f = boost::bind(&Body_Manager::_callbackDynamicROSParam, this, _1, _2);
  server.setCallback(f);
  ROS_INFO("START SERVER");

  udp.InitCmdData(_udp_low_cmd);
}

Body_Manager::~Body_Manager()
{
  delete _legController;
  delete _stateEstimator;
}

void Body_Manager::UDPRecv() { udp.Recv(); }

void Body_Manager::UDPSend() { udp.Send(); }

UNITREE_LEGGED_SDK::LowCmd Body_Manager::_rosCmdToUdp(unitree_legged_msgs::LowCmd ros_low_cmd)
{
  UNITREE_LEGGED_SDK::LowCmd udp_low_cmd;

  udp_low_cmd.levelFlag = ros_low_cmd.levelFlag;

  for (size_t i = 0; i < 12; i++)
  {
    udp_low_cmd.motorCmd[i].mode = ros_low_cmd.motorCmd[i].mode;
    udp_low_cmd.motorCmd[i].q = ros_low_cmd.motorCmd[i].q;
    udp_low_cmd.motorCmd[i].dq = ros_low_cmd.motorCmd[i].dq;
    udp_low_cmd.motorCmd[i].Kp = ros_low_cmd.motorCmd[i].Kp;
    udp_low_cmd.motorCmd[i].Kd = ros_low_cmd.motorCmd[i].Kd;
    udp_low_cmd.motorCmd[i].tau = ros_low_cmd.motorCmd[i].tau;
  }

  return udp_low_cmd;
}

unitree_legged_msgs::LowState Body_Manager::_udpStateToRos(UNITREE_LEGGED_SDK::LowState udp_low_state)
{
  unitree_legged_msgs::LowState ros_low_state;

  for (size_t i = 0; i < 12; i++)
  {
    ros_low_state.motorState[i].mode = udp_low_state.motorState[i].mode;
    ros_low_state.motorState[i].q = udp_low_state.motorState[i].q;
    ros_low_state.motorState[i].dq = udp_low_state.motorState[i].dq;
    ros_low_state.motorState[i].ddq = udp_low_state.motorState[i].ddq;
    ros_low_state.motorState[i].q_raw = udp_low_state.motorState[i].q_raw;
    ros_low_state.motorState[i].dq_raw = udp_low_state.motorState[i].dq_raw;
    ros_low_state.motorState[i].ddq_raw = udp_low_state.motorState[i].ddq_raw;
    ros_low_state.motorState[i].tauEst = udp_low_state.motorState[i].tauEst;
    ros_low_state.motorState[i].temperature = udp_low_state.motorState[i].temperature;
  }

  ros_low_state.imu.accelerometer[0] = udp_low_state.imu.accelerometer[0];
  ros_low_state.imu.accelerometer[1] = udp_low_state.imu.accelerometer[1];
  ros_low_state.imu.accelerometer[2] = udp_low_state.imu.accelerometer[2];

  ros_low_state.imu.gyroscope[0] = udp_low_state.imu.gyroscope[0];
  ros_low_state.imu.gyroscope[1] = udp_low_state.imu.gyroscope[1];
  ros_low_state.imu.gyroscope[2] = udp_low_state.imu.gyroscope[2];

  ros_low_state.imu.quaternion[0] = udp_low_state.imu.quaternion[0];
  ros_low_state.imu.quaternion[1] = udp_low_state.imu.quaternion[1];
  ros_low_state.imu.quaternion[2] = udp_low_state.imu.quaternion[2];
  ros_low_state.imu.quaternion[3] = udp_low_state.imu.quaternion[3];

  ros_low_state.imu.temperature = udp_low_state.imu.temperature;

  return ros_low_state;
}

void Body_Manager::init()
{
  _initSubscribers();
  _initPublishers();
  _initParameters();

  _time_start = ros::Time::now();

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

  // TODO: check exist
  if (_is_param_updated)
  {
    ROS_INFO_STREAM("Get params from dynamic reconfigure");
  }
  else
  {
    ROS_WARN_STREAM("No dynamic config data");
  }

  _quadruped = buildMiniCheetah<float>();

  controlParameters.controller_dt = 1.0 / (double)MAIN_LOOP_RATE;
  cout << controlParameters.controller_dt << " dt" << endl;

  // Initialize the model and robot data
  _model = _quadruped.buildModel();

  _debug = new Debug(_time_start);

  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(&vectorNavData, _legController->datas, &footContactState, &_stateEstimate, &controlParameters);
  initializeStateEstimator();

  // Initialize the DesiredStateCommand object
  _desiredStateCommand = new DesiredStateCommand<float>(&driverCommand, &controlParameters, &_stateEstimate, controlParameters.controller_dt);

  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<float>(&_rosParameters, controlParameters.controller_dt);

  // Initializes the Control FSM with all the
  // required data
  _controlFSM = new ControlFSM<float>(&_quadruped, _stateEstimator, _legController, _gaitScheduler,
                                      _desiredStateCommand, &controlParameters, &_rosParameters, _debug);

  controlParameters.control_mode = 0;
}

void Body_Manager::_readRobotData()
{
  // TODO check if we can send only zero struct and recieve falid data and dont crash robot
  // controller
  udp.SetSend(_udp_low_cmd);
  // TODO check if TRULY NEW data recieved
  udp.GetRecv(_udp_low_state);

  _low_state = _udpStateToRos(_udp_low_state);
  _pub_low_state.publish(_low_state);

  for (uint8_t leg_num = 0; leg_num < 4; leg_num++)
  {
    spiData.q_abad[leg_num] = _low_state.motorState[leg_num * 3 + 0].q;
    spiData.q_hip[leg_num] = -_low_state.motorState[leg_num * 3 + 1].q;
    spiData.q_knee[leg_num] = -_low_state.motorState[leg_num * 3 + 2].q;

    spiData.qd_abad[leg_num] = _low_state.motorState[leg_num * 3 + 0].dq;
    spiData.qd_hip[leg_num] = -_low_state.motorState[leg_num * 3 + 1].dq;
    spiData.qd_knee[leg_num] = -_low_state.motorState[leg_num * 3 + 2].dq;
  }

  is_stand = _low_state.levelFlag;

  vectorNavData.gyro[0] = _low_state.imu.gyroscope[0];
  vectorNavData.gyro[1] = _low_state.imu.gyroscope[1];
  vectorNavData.gyro[2] = _low_state.imu.gyroscope[2];

  vectorNavData.accelerometer[0] = _low_state.imu.accelerometer[0];
  vectorNavData.accelerometer[1] = _low_state.imu.accelerometer[1];
  vectorNavData.accelerometer[2] = _low_state.imu.accelerometer[2];

  vectorNavData.quat[0] = _low_state.imu.quaternion[0]; // w
  vectorNavData.quat[1] = _low_state.imu.quaternion[1]; // x
  vectorNavData.quat[2] = _low_state.imu.quaternion[2]; // y
  vectorNavData.quat[3] = _low_state.imu.quaternion[3]; // z

  // Датчики контакта на лапах
  // for (size_t leg = 0; leg < 4; leg++)
  // {
  //   footContactState(leg) = _low_state.footForce[leg];
  //   //    if
  //   //    ((_stateEstimator->getResult().contactEstimate[leg]
  //   //    <= 0.001) &&
  //   //      (footContactState(leg) == 1) )
  //   //    {
  //   ////      std::cout << "EARLY CONTACT" <<
  //   /// std::endl;

  //   //    }
  // }

  // _stateEstimator->setContactSensorData(footContactState);
}

void Body_Manager::run()
{
  Vec4<float> contact_states(_low_state.footForce[0], _low_state.footForce[1],
                             _low_state.footForce[2], _low_state.footForce[3]);

  if (is_udp_connection)
  {
    _readRobotData();
  }

  // Run the state estimator step
  _stateEstimator->run();

  // Update the data from the robot (put data from LowState to LegController->Datas)
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
    // Find the current gait schedule
    _gaitScheduler->step();

    // Find the desired state trajectory
    _desiredStateCommand->convertToStateCommands();

    // Run the Control FSM code
    _controlFSM->runFSM();
  }

  // Sets the leg controller commands for the robot appropriate commands
  finalizeStep();

#ifdef FSM_AUTO
  //оставляю эту часть для автоматического
  //перехода между режимами, если понадобится
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
#endif

  _debug->updateVisualization();
  _debug->updatePlot();
  _debug->tfPublish();
}

void Body_Manager::setupStep()
{
  _legController->updateData(&spiData);

  // Setup the leg controller for a new iteration
  _legController->zeroCommand(); //нельзя убирать
  _legController->setEnabled(true);
  _legController->is_low_level = _is_low_level;

  // todo safety checks, sanity checks, etc...
}

void Body_Manager::finalizeStep()
{
  ros::Duration delta_t = ros::Time::now() - _time_start;

  _legController->updateCommand(&spiCommand);
  _iterations++;

  _debug->all_legs_info.header.stamp = _zero_time + delta_t;

  //put actual body info
  _debug->body_info.pos_act.x = _stateEstimator->getResult().position.x();
  _debug->body_info.pos_act.y = _stateEstimator->getResult().position.y();
  _debug->body_info.pos_act.z = _stateEstimator->getResult().position.z();

  _debug->body_info.vel_act.linear.x = _stateEstimator->getResult().vBody[0];
  _debug->body_info.vel_act.linear.y = _stateEstimator->getResult().vBody[1];
  _debug->body_info.vel_act.linear.z = _stateEstimator->getResult().vBody[2];

  _debug->body_info.quat_act.x = _stateEstimator->getResult().orientation.x();
  _debug->body_info.quat_act.y = _stateEstimator->getResult().orientation.y();
  _debug->body_info.quat_act.z = _stateEstimator->getResult().orientation.z();
  _debug->body_info.quat_act.w = _stateEstimator->getResult().orientation.w();

  _debug->body_info.euler_act.x = _stateEstimator->getResult().rpy[0];
  _debug->body_info.euler_act.y = _stateEstimator->getResult().rpy[1];
  _debug->body_info.euler_act.z = _stateEstimator->getResult().rpy[2];

  _debug->body_info.vel_act.angular.x = _stateEstimator->getResult().omegaBody[0];
  _debug->body_info.vel_act.angular.y = _stateEstimator->getResult().omegaBody[1];
  _debug->body_info.vel_act.angular.z = _stateEstimator->getResult().omegaBody[2];

  //put actual q and dq in debug class
  for (size_t leg_num = 0; leg_num < 4; leg_num++)
  {
    _debug->all_legs_info.leg.at(leg_num).joint.at(0).q = spiData.q_abad[leg_num];
    _debug->all_legs_info.leg.at(leg_num).joint.at(1).q = -spiData.q_hip[leg_num];
    _debug->all_legs_info.leg.at(leg_num).joint.at(2).q = -spiData.q_knee[leg_num];

    _debug->all_legs_info.leg.at(leg_num).joint.at(0).dq = spiData.qd_abad[leg_num];
    _debug->all_legs_info.leg.at(leg_num).joint.at(1).dq = -spiData.qd_hip[leg_num];
    _debug->all_legs_info.leg.at(leg_num).joint.at(2).dq = -spiData.qd_knee[leg_num];

    _debug->all_legs_info.leg.at(leg_num).joint.at(0).Kp_joint = spiCommand.kp_abad[leg_num];
    _debug->all_legs_info.leg.at(leg_num).joint.at(1).Kp_joint = spiCommand.kp_hip[leg_num];
    _debug->all_legs_info.leg.at(leg_num).joint.at(2).Kp_joint = spiCommand.kp_knee[leg_num];

    _debug->all_legs_info.leg.at(leg_num).joint.at(0).Kd_joint = spiCommand.kd_abad[leg_num];
    _debug->all_legs_info.leg.at(leg_num).joint.at(1).Kd_joint = spiCommand.kd_hip[leg_num];
    _debug->all_legs_info.leg.at(leg_num).joint.at(2).Kd_joint = spiCommand.kd_knee[leg_num];

    _debug->all_legs_info.leg.at(leg_num).p_act.x = _legController->datas[leg_num].p[0];
    _debug->all_legs_info.leg.at(leg_num).p_act.y = _legController->datas[leg_num].p[1];
    _debug->all_legs_info.leg.at(leg_num).p_act.z = _legController->datas[leg_num].p[2];

    _debug->all_legs_info.leg.at(leg_num).v_act.x = _legController->datas[leg_num].v[0];
    _debug->all_legs_info.leg.at(leg_num).v_act.y = _legController->datas[leg_num].v[1];
    _debug->all_legs_info.leg.at(leg_num).v_act.z = _legController->datas[leg_num].v[2];

    if (controlParameters.control_mode != K_LOCOMOTION)
    {
      _debug->all_legs_info.leg.at(leg_num).p_des.x = _legController->commands[leg_num].pDes[0];
      _debug->all_legs_info.leg.at(leg_num).p_des.y = _legController->commands[leg_num].pDes[1];
      _debug->all_legs_info.leg.at(leg_num).p_des.z = _legController->commands[leg_num].pDes[2];

      _debug->all_legs_info.leg.at(leg_num).v_des.x = _legController->commands[leg_num].vDes[0];
      _debug->all_legs_info.leg.at(leg_num).v_des.y = _legController->commands[leg_num].vDes[1];
      _debug->all_legs_info.leg.at(leg_num).v_des.z = _legController->commands[leg_num].vDes[2];
    }
  }

  for (int i = 0; i < 4; i++)
  {
    _torqueCalculator(&spiCommand, &spiData, &_spi_torque, i);
  }

  uint8_t mode[4] = {MOTOR_BREAK};

  for (size_t i = 0; i < 4; i++)
  {
    if (_legController->_legEnabled[i])
    {
      mode[i] = MOTOR_ON;
    }
    else
    {
      mode[i] = MOTOR_BREAK;
    }
  }

  _low_cmd.header.stamp = _zero_time + delta_t;

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    // if is low level == false -> tau control
    if (_legController->is_low_level == false)
    {
      for (uint8_t servo_num = 0; servo_num < 3; servo_num++)
      {
        _low_cmd.motorCmd[leg * 3 + servo_num].mode = mode[leg];
        _low_cmd.motorCmd[leg * 3 + servo_num].q = PosStopF;
        _low_cmd.motorCmd[leg * 3 + servo_num].dq = VelStopF;
        _low_cmd.motorCmd[leg * 3 + servo_num].Kp = 0;
        _low_cmd.motorCmd[leg * 3 + servo_num].Kd = 0;
      }
    }
    else
    {
      _low_cmd.motorCmd[leg * 3 + 0].mode = mode[leg];
      _low_cmd.motorCmd[leg * 3 + 0].q = spiCommand.q_des_abad[leg];
      _low_cmd.motorCmd[leg * 3 + 0].dq = spiCommand.qd_des_abad[leg];
      _low_cmd.motorCmd[leg * 3 + 0].Kp = spiCommand.kp_abad[leg];
      _low_cmd.motorCmd[leg * 3 + 0].Kd = spiCommand.kd_abad[leg];

      _low_cmd.motorCmd[leg * 3 + 1].mode = mode[leg];
      _low_cmd.motorCmd[leg * 3 + 1].q = spiCommand.q_des_hip[leg];
      _low_cmd.motorCmd[leg * 3 + 1].dq = spiCommand.qd_des_hip[leg];
      _low_cmd.motorCmd[leg * 3 + 1].Kp = spiCommand.kp_hip[leg];
      _low_cmd.motorCmd[leg * 3 + 1].Kd = spiCommand.kd_hip[leg];

      _low_cmd.motorCmd[leg * 3 + 2].mode = mode[leg];
      _low_cmd.motorCmd[leg * 3 + 2].q = spiCommand.q_des_knee[leg];
      _low_cmd.motorCmd[leg * 3 + 2].dq = spiCommand.qd_des_knee[leg];
      _low_cmd.motorCmd[leg * 3 + 2].Kp = spiCommand.kp_knee[leg];
      _low_cmd.motorCmd[leg * 3 + 2].Kd = spiCommand.kd_knee[leg];
    }
  }

  for (uint8_t leg_num = 0; leg_num < 4; leg_num++)
  {
    _low_cmd.motorCmd[leg_num * 3 + 0].tau = _spi_torque.tau_abad[leg_num];
    _low_cmd.motorCmd[leg_num * 3 + 1].tau = -_spi_torque.tau_hip[leg_num];
    _low_cmd.motorCmd[leg_num * 3 + 2].tau = -_spi_torque.tau_knee[leg_num];
  }

  _low_cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;

  // only for real robot
  if (is_udp_connection)
  {
    // convert ros struct ro udp struct
    _udp_low_cmd = _rosCmdToUdp(_low_cmd);
    // position limit safety check
    safe.PositionLimit(_udp_low_cmd);
    // power protection safety check
    safe.PowerProtect(_udp_low_cmd, _udp_low_state, 5);

    // put udp struct to udp send transfer process
    udp.SetSend(_udp_low_cmd);
  }

  _pub_low_cmd.publish(_low_cmd);
  // t1.start();
  // ROS_INFO("pub");
}

/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void Body_Manager::initializeStateEstimator()
{
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  _stateEstimator->getResult();

  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);

  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
}

void Body_Manager::_initSubscribers()
{
  _sub_low_state = _nh.subscribe("/low_state", 1, &Body_Manager::_lowStateCallback, this,
                                 ros::TransportHints().tcpNoDelay(true));
  _sub_cmd_vel = _nh.subscribe("/cmd_vel", 1, &Body_Manager::_cmdVelCallback, this,
                               ros::TransportHints().tcpNoDelay(true));
}

void Body_Manager::_initPublishers()
{
  _pub_low_cmd = _nh.advertise<unitree_legged_msgs::LowCmd>("/low_cmd", 1);
  _pub_low_state = _nh.advertise<unitree_legged_msgs::LowState>("/low_state", 1);
}

void Body_Manager::_lowStateCallback(unitree_legged_msgs::LowState msg)
{
  // cout << "timer ns: " << t1.getNs() << endl;
  // cout << "timer ms: " << t1.getMs() << endl;
  // ROS_INFO("calback");

  _low_state = msg;

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

  vectorNavData.quat[0] = msg.imu.quaternion[0]; // w
  vectorNavData.quat[1] = msg.imu.quaternion[1]; // x
  vectorNavData.quat[2] = msg.imu.quaternion[2]; // y
  vectorNavData.quat[3] = msg.imu.quaternion[3]; // z

  // Датчики контакта на лапах
  for (size_t leg = 0; leg < 4; leg++)
  {
    footContactState(leg) = msg.footForce[leg];
    //    if
    //    ((_stateEstimator->getResult().contactEstimate[leg]
    //    <= 0.001) &&
    //      (footContactState(leg) == 1) )
    //    {
    ////      std::cout << "EARLY CONTACT" <<
    /// std::endl;

    //    }
  }

  _stateEstimator->setContactSensorData(footContactState);

  // Фильтрация данных
  //  _filterInput();
}

void Body_Manager::_cmdVelCallback(geometry_msgs::Twist msg)
{
  driverCommand.leftStickAnalog[1] = msg.linear.x;
  driverCommand.leftStickAnalog[0] = -msg.linear.y;

  driverCommand.rightStickAnalog[1] = msg.angular.x;
  driverCommand.rightStickAnalog[0] = -msg.angular.z;
}

/*!
 * Emulate the spi board to estimate the torque.
 */
void Body_Manager::_torqueCalculator(SpiCommand* cmd, SpiData* data, spi_torque_t* torque_out,
                                     int board_num)
{
  if (_legController->is_low_level == false)
  {
    torque_out->tau_abad[board_num] =
        cmd->kp_abad[board_num] * (cmd->q_des_abad[board_num] - data->q_abad[board_num]) +
        cmd->kd_abad[board_num] * (cmd->qd_des_abad[board_num] - data->qd_abad[board_num]) +
        cmd->tau_abad_ff[board_num];

    torque_out->tau_hip[board_num] =
        cmd->kp_hip[board_num] * (cmd->q_des_hip[board_num] - data->q_hip[board_num]) +
        cmd->kd_hip[board_num] * (cmd->qd_des_hip[board_num] - data->qd_hip[board_num]) +
        cmd->tau_hip_ff[board_num];

    torque_out->tau_knee[board_num] =
        cmd->kp_knee[board_num] * (cmd->q_des_knee[board_num] - data->q_knee[board_num]) +
        cmd->kd_knee[board_num] * (cmd->qd_des_knee[board_num] - data->qd_knee[board_num]) +
        cmd->tau_knee_ff[board_num];
  }
  else
  {
    torque_out->tau_abad[board_num] = cmd->tau_abad_ff[board_num];

    torque_out->tau_hip[board_num] = cmd->tau_hip_ff[board_num];

    torque_out->tau_knee[board_num] = cmd->tau_knee_ff[board_num];
  }

  const float safe_torque[3] = {4.f, 4.f, 4.f};
  const float max_torque[3] = {17.f, 17.f, 26.f};
  const float* torque_limits;

  if (_is_torque_safe)
  {
    torque_limits = safe_torque;
  }
  else
  {
    torque_limits = max_torque;
  }

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
}

void Body_Manager::_initParameters()
{
  readRosParam(ros::this_node::getName() + "/is_low_level", _is_low_level);
  readRosParam(ros::this_node::getName() + "/torque_safe_limit", _is_torque_safe);
  readRosParam(ros::this_node::getName() + "/udp_connection", is_udp_connection);
}

void Body_Manager::_callbackDynamicROSParam(be2r_cmpc_unitree::ros_dynamic_paramsConfig& config, uint32_t level)
{
  (void)level;
  _is_param_updated = true;
  _rosParameters = config;
  controlParameters.control_mode = config.FSM_State;

  ROS_INFO_STREAM("New dynamic data!");
}

void Body_Manager::_filterInput() {}
