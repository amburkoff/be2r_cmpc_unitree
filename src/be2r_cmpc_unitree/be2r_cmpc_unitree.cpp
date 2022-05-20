#include "be2r_cmpc_unitree.hpp"

using namespace std;

Body_Manager::Body_Manager()
  : _zero_time(0)
{
  footContactState = Vec4<uint8_t>::Zero();
  f = boost::bind(&Body_Manager::_callbackDynamicROSParam, this, _1, _2);
  server.setCallback(f);
  ROS_INFO("START SERVER");
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
  _initParameters();

  _time_start = ros::Time::now();

  printf("[Hardware Bridge] Loading parameters "
         "from file...\n");

  try
  {
    controlParameters.initializeFromYamlFile(THIS_COM "config/mini-cheetah-defaults.yaml");
  }
  catch (std::exception& e)
  {
    printf("Failed to initialize robot "
           "parameters from yaml file: %s\n",
           e.what());
    exit(1);
  }

  if (!controlParameters.isFullyInitialized())
  {
    printf("Failed to initialize all robot "
           "parameters\n");
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

  // Always initialize the leg controller and
  // state entimator
  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(
    &vectorNavData, _legController->datas, &footContactState, &_stateEstimate, &controlParameters);
  initializeStateEstimator();

  // Initialize the DesiredStateCommand object
  _desiredStateCommand = new DesiredStateCommand<float>(
    &driverCommand, &controlParameters, &_stateEstimate, controlParameters.controller_dt);

  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<float>(&_rosParameters, controlParameters.controller_dt);

  // Initializes the Control FSM with all the
  // required data
  _controlFSM = new ControlFSM<float>(&_quadruped, _stateEstimator, _legController, _gaitScheduler,
                                      _desiredStateCommand, &controlParameters, &_rosParameters);

  _leg_contoller_params[0].zero();
  _leg_contoller_params[1].zero();
  _leg_contoller_params[2].zero();
  _leg_contoller_params[3].zero();

  controlParameters.control_mode = 0;
}

void Body_Manager::run()
{
  Vec4<float> contact_states(_low_state.footForce[0], _low_state.footForce[1],
                             _low_state.footForce[2], _low_state.footForce[3]);

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

  // Sets the leg controller commands for the
  // robot appropriate commands
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

  _updateVisualization();
  _updatePlot();
  _tfPublish();
}

void Body_Manager::setupStep()
{
  _legController->updateData(&spiData);

  // Setup the leg controller for a new iteration
  _legController->zeroCommand(); //нельзя убирать
  _legController->setEnabled(true);

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
  uint8_t mode[4] = { MOTOR_BREAK };

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

  ros::Duration delta_t = ros::Time::now() - _time_start;
  // _low_cmd.header.stamp = ros::Time::now();
  _low_cmd.header.stamp = _zero_time + delta_t;

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    //if is low level == false -> tau control
    if (_legController->commands[leg].is_low_level == false)
    {
      for (uint8_t servo_num = 0; servo_num < 3; servo_num++)
      {
        _low_cmd.motorCmd[leg * 3 + servo_num].mode = mode[leg];
        _low_cmd.motorCmd[leg * 3 + servo_num].q = PosStopF;
        _low_cmd.motorCmd[leg * 3 + servo_num].dq = VelStopF;
      }
    }
    else
    {
      //if is low level == true -> joint control
      // for (uint8_t servo_num = 0; servo_num < 3; servo_num++)
      // {
      //   _low_cmd.motorCmd[leg * 3 + servo_num].mode = mode[leg];
      //   _low_cmd.motorCmd[leg * 3 + servo_num].q = _legController->commands[leg].qDes(servo_num);
      //   _low_cmd.motorCmd[leg * 3 + servo_num].dq = _legController->commands[leg].qdDes(servo_num);
      //   _low_cmd.motorCmd[leg * 3 + servo_num].Kp = _legController->commands[leg].kpJoint(servo_num, servo_num);
      //   _low_cmd.motorCmd[leg * 3 + servo_num].Kd = _legController->commands[leg].kdJoint(servo_num, servo_num);
      //   _low_cmd.motorCmd[leg * 3 + servo_num].tau = 0;
      // }
      _low_cmd.motorCmd[leg * 3 + 0].mode = mode[leg];
      _low_cmd.motorCmd[leg * 3 + 0].q = spiCommand.q_des_abad[leg];
      _low_cmd.motorCmd[leg * 3 + 0].dq = spiCommand.qd_des_abad[leg];
      _low_cmd.motorCmd[leg * 3 + 0].Kp = spiCommand.kp_abad[leg];
      _low_cmd.motorCmd[leg * 3 + 0].Kd = spiCommand.kd_abad[leg];
      _low_cmd.motorCmd[leg * 3 + 0].tau = 0;
      
      _low_cmd.motorCmd[leg * 3 + 1].mode = mode[leg];
      _low_cmd.motorCmd[leg * 3 + 1].q = spiCommand.q_des_hip[leg];
      _low_cmd.motorCmd[leg * 3 + 1].dq = spiCommand.qd_des_hip[leg];
      _low_cmd.motorCmd[leg * 3 + 1].Kp = spiCommand.kp_hip[leg];
      _low_cmd.motorCmd[leg * 3 + 1].Kd = spiCommand.kd_hip[leg];
      _low_cmd.motorCmd[leg * 3 + 1].tau = 0;

      _low_cmd.motorCmd[leg * 3 + 2].mode = mode[leg];
      _low_cmd.motorCmd[leg * 3 + 2].q = spiCommand.q_des_knee[leg];
      _low_cmd.motorCmd[leg * 3 + 2].dq = spiCommand.qd_des_knee[leg];
      _low_cmd.motorCmd[leg * 3 + 2].Kp = spiCommand.kp_knee[leg];
      _low_cmd.motorCmd[leg * 3 + 2].Kd = spiCommand.kd_knee[leg];
      _low_cmd.motorCmd[leg * 3 + 2].tau = 0;
    // spiCommand->q_des_abad[leg] = commands[leg].qDes(0);
    }
  }

  // cout << "servo " << (int)0 << ": " << _legController->commands[0].kpJoint << endl;

  for (uint8_t leg_num = 0; leg_num < 4; leg_num++)
  {
    // if (_legController->commands[leg_num].is_low_level == false)
    // {
    _low_cmd.motorCmd[leg_num * 3 + 0].tau = _spi_torque.tau_abad[leg_num];
    _low_cmd.motorCmd[leg_num * 3 + 1].tau = -_spi_torque.tau_hip[leg_num];
    _low_cmd.motorCmd[leg_num * 3 + 2].tau = -_spi_torque.tau_knee[leg_num];
    // }
  }
  // DEBUG
  for (size_t i = 0; i < 4; i++)
  {
    _low_cmd.ff[i].x = _stateEstimator->getResult().swingProgress[i];
    _low_cmd.ff[i].y = _stateEstimator->getResult().contactEstimate[i];
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
  _pub_joint_states = _nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
  _pub_state_error = _nh.advertise<unitree_legged_msgs::StateError>("/state_error", 1);
  _pub_leg_error = _nh.advertise<unitree_legged_msgs::LegError>("/leg_error", 1);
  _pub_parameters = _nh.advertise<unitree_legged_msgs::Parameters>("/parameters", 1);
}

void Body_Manager::_lowStateCallback(unitree_legged_msgs::LowState msg)
{
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
  if (_legController->commands[board_num].is_low_level == false)
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
  }
  else
  {
    torque_out->tau_abad[board_num] = cmd->tau_abad_ff[board_num];

    torque_out->tau_hip[board_num] = cmd->tau_hip_ff[board_num];

    torque_out->tau_knee[board_num] = cmd->tau_knee_ff[board_num];
  }

  const float safe_torque[3] = {4.f, 4.f, 4.f};
  const float max_torque[3] = {17.f, 17.f, 26.f};

#ifdef TORQUE_LIMIT_SAFE
  const float* torque_limits = safe_torque;
#endif

#ifdef TORQUE_LIMIT_MAX
  const float* torque_limits = max_torque;
#endif

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

  // float q0_e = cmd->q_des_abad[board_num] -
  // data->q_abad[board_num]; float q1_e =
  // cmd->q_des_hip[board_num] -
  // data->q_hip[board_num]; float q2_e =
  // cmd->q_des_knee[board_num] -
  // data->q_knee[board_num];

  // cout << "Leg: " << board_num << " q0_e: " <<
  // q0_e << " q1_e: " << q1_e << " q2_e: " <<
  // q2_e << endl; cout << "Leg: " << board_num <<
  // " t0: " << torque_out->tau_abad[board_num] <<
  // " t1: " << torque_out->tau_hip[board_num] <<
  // " t2: " << torque_out->tau_knee[board_num] <<
  // endl;
}

void Body_Manager::_initParameters()
{
  //  readRosParam("/Swing_traj_height", userParameters.Swing_traj_height);
  //  readRosParam("/cmpc_x_drag", userParameters.cmpc_x_drag);
  //  readRosParam("/cmpc_use_sparse", userParameters.cmpc_use_sparse);
  //  readRosParam("/cmpc_bonus_swing", userParameters.cmpc_bonus_swing);
  //  readRosParam("/use_jcqp", userParameters.use_jcqp);
  //  readRosParam("/jcqp_max_iter", userParameters.jcqp_max_iter);
  //  readRosParam("/jcqp_rho", userParameters.jcqp_rho);
  //  readRosParam("/jcqp_sigma", userParameters.jcqp_sigma);
  //  readRosParam("/jcqp_alpha", userParameters.jcqp_alpha);
  //  readRosParam("/jcqp_terminate", userParameters.jcqp_terminate);
  //  readRosParam("/gait_type", userParameters.gait_type);
  //  readRosParam("/gait_period_time", userParameters.gait_period_time);
  //  readRosParam("/gait_switching_phase", userParameters.gait_switching_phase);
  //  readRosParam("/gait_override", userParameters.gait_override);
  //  readRosParam("/stance_legs", userParameters.stance_legs);
}

void Body_Manager::_updateVisualization()
{
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.name.push_back("FR_hip_joint");
  msg.name.push_back("FR_thigh_joint");
  msg.name.push_back("FR_calf_joint");

  msg.name.push_back("FL_hip_joint");
  msg.name.push_back("FL_thigh_joint");
  msg.name.push_back("FL_calf_joint");

  msg.name.push_back("RR_hip_joint");
  msg.name.push_back("RR_thigh_joint");
  msg.name.push_back("RR_calf_joint");

  msg.name.push_back("RL_hip_joint");
  msg.name.push_back("RL_thigh_joint");
  msg.name.push_back("RL_calf_joint");

  for (uint8_t joint_num = 0; joint_num < 12; joint_num++)
  {
    msg.position.push_back(_low_state.motorState[joint_num].q);
    msg.velocity.push_back(_low_state.motorState[joint_num].dq);
  }

  _pub_joint_states.publish(msg);
}

void Body_Manager::_tfPublish()
{
  geometry_msgs::TransformStamped odom_trans;

  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base";

  odom_trans.transform.translation.x = _stateEstimator->getResult().position.x();
  odom_trans.transform.translation.y = _stateEstimator->getResult().position.y();
  odom_trans.transform.translation.z = _stateEstimator->getResult().position.z();

  geometry_msgs::Quaternion odom_quat;
  // TODO почему результаты естиматора приходится менять местами?
  odom_quat.x = _stateEstimator->getResult().orientation.y();
  odom_quat.y = _stateEstimator->getResult().orientation.z();
  odom_quat.z = _stateEstimator->getResult().orientation.w();
  odom_quat.w = _stateEstimator->getResult().orientation.x();
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster.sendTransform(odom_trans);
}

void Body_Manager::_updatePlot()
{
  unitree_legged_msgs::StateError msg;

  ros::Duration delta_t = ros::Time::now() - _time_start;
  // msg.header.stamp = ros::Time::now();
  msg.header.stamp = _zero_time + delta_t;

  static float x_vel_cmd, y_vel_cmd, yaw_turn_rate, x_vel_des, y_vel_des, yaw_des, roll_des,
    pitch_des;
  float filter = 0.1;
  float dt = controlParameters.controller_dt;

  yaw_turn_rate = _desiredStateCommand->rightAnalogStick[0];
  x_vel_cmd = _desiredStateCommand->leftAnalogStick[1];
  y_vel_cmd = _desiredStateCommand->leftAnalogStick[0];

  Eigen::Vector3f v_act;
  Eigen::Vector3f w_act;
  v_act = _stateEstimator->getResult().vBody;
  w_act = _stateEstimator->getResult().omegaBody;

  // float* p = seResult.position.data();
  // float* v = seResult.vWorld.data();
  // float* w = seResult.omegaWorld.data();
  // float* q = seResult.orientation.data();

  // msg.pose.position.x = _stateEstimator->getResult().position

  yaw_des = _stateEstimator->getResult().rpy[2] + dt * yaw_turn_rate;
  roll_des = 0.;
  pitch_des = 0.;
  x_vel_des = x_vel_des * (1 - filter) + x_vel_cmd * filter;
  y_vel_des = y_vel_des * (1 - filter) + y_vel_cmd * filter;

  msg.pose.orientation.x = 0 - _stateEstimator->getResult().rpy[0];
  msg.pose.orientation.y = 0 - _stateEstimator->getResult().rpy[1];
  msg.pose.orientation.z = yaw_des - _stateEstimator->getResult().rpy[2];

  msg.twist.linear.x = x_vel_des - v_act(0);
  msg.twist.linear.y = y_vel_des - v_act(1);
  msg.twist.linear.z = 0 - v_act(2);

  msg.twist.angular.x = roll_des - w_act(0);
  msg.twist.angular.y = pitch_des - w_act(1);
  msg.twist.angular.z = yaw_turn_rate - w_act(2);

  _pub_state_error.publish(msg);

  unitree_legged_msgs::LegError leg_error;

  leg_error.header = msg.header;

  // for all legs
  for (size_t i = 0; i < 4; i++)
  {
    // p actual
    leg_error.p_act[i].x = _legController->datas[i].p(0);
    leg_error.p_act[i].y = _legController->datas[i].p(1);
    leg_error.p_act[i].z = _legController->datas[i].p(2);

    // v actual
    leg_error.v_act[i].x = _legController->datas[i].v(0);
    leg_error.v_act[i].y = _legController->datas[i].v(1);
    leg_error.v_act[i].z = _legController->datas[i].v(2);

    // p desired
    leg_error.p_des[i].x = _legController->commands[i].pDes(0);
    leg_error.p_des[i].y = _legController->commands[i].pDes(1);
    leg_error.p_des[i].z = _legController->commands[i].pDes(2);

    // v desired
    leg_error.v_des[i].x = _legController->commands[i].vDes(0);
    leg_error.v_des[i].y = _legController->commands[i].vDes(1);
    leg_error.v_des[i].z = _legController->commands[i].vDes(2);

    // p error
    leg_error.p_error[i].x = _legController->commands[i].pDes(0) - _legController->datas[i].p(0);
    leg_error.p_error[i].y = _legController->commands[i].pDes(1) - _legController->datas[i].p(1);
    leg_error.p_error[i].z = _legController->commands[i].pDes(2) - _legController->datas[i].p(2);

    // v error
    leg_error.v_error[i].x = _legController->commands[i].vDes(0) - _legController->datas[i].v(0);
    leg_error.v_error[i].y = _legController->commands[i].vDes(1) - _legController->datas[i].v(1);
    leg_error.v_error[i].z = _legController->commands[i].vDes(2) - _legController->datas[i].v(2);

    //q des
    leg_error.q_des[i].x = spiCommand.q_des_abad[i];
    leg_error.q_des[i].y = spiCommand.q_des_hip[i];
    leg_error.q_des[i].z = spiCommand.q_des_knee[i];
    // leg_error.q_des[i].x = _legController->commands[i].qDes(0);
    // leg_error.q_des[i].y = _legController->commands[i].qDes(1);
    // leg_error.q_des[i].z = _legController->commands[i].qDes(2);

    //dq des
    leg_error.dq_des[i].x = spiCommand.qd_des_abad[i];
    leg_error.dq_des[i].y = spiCommand.qd_des_hip[i];
    leg_error.dq_des[i].z = spiCommand.qd_des_knee[i];
  }

  _pub_leg_error.publish(leg_error);

  unitree_legged_msgs::Parameters param_msg;

  param_msg.header = msg.header;

  param_msg.Kp_cartesian.x = _legController->commands[0].kpCartesian(0);
  param_msg.Kp_cartesian.y = _legController->commands[0].kpCartesian(4);
  param_msg.Kp_cartesian.z = _legController->commands[0].kpCartesian(8);

  param_msg.Kd_cartesian.x = _legController->commands[0].kdCartesian(0);
  param_msg.Kd_cartesian.y = _legController->commands[0].kdCartesian(4);
  param_msg.Kd_cartesian.z = _legController->commands[0].kdCartesian(8);

  param_msg.saturation = _legController->commands[0].i_saturation;

  _pub_parameters.publish(param_msg);
}

void Body_Manager::_callbackDynamicROSParam(be2r_cmpc_unitree::ros_dynamic_paramsConfig& config,
                                            uint32_t level)
{
  _is_param_updated = true;
  _rosParameters = config;
  controlParameters.control_mode = config.FSM_State;
  //  userParameters.use_wbc = config.use_wbc;
  //  userParameters.Swing_Kp_cartesian =
  //    Vec3<double>(config.Kp_cartesian_0, config.Kp_cartesian_1, config.Kp_cartesian_2);
  //  userParameters.Swing_Kd_cartesian =
  //    Vec3<double>(config.Kd_cartesian_0, config.Kd_cartesian_1, config.Kd_cartesian_2);
  //  userParameters.Kp_joint = Vec3<double>(config.Kp_joint_0, config.Kp_joint_1,
  //  config.Kp_joint_2); userParameters.Kd_joint = Vec3<double>(config.Kd_joint_0,
  //  config.Kd_joint_1, config.Kd_joint_2); userParameters.Kp_ori = Vec3<double>(config.Kp_ori_0,
  //  config.Kp_ori_1, config.Kp_ori_2); userParameters.Kd_ori = Vec3<double>(config.Kd_ori_0,
  //  config.Kd_ori_1, config.Kd_ori_2); userParameters.Kp_body = Vec3<double>(config.Kp_body_0,
  //  config.Kp_body_1, config.Kp_body_2); userParameters.Kd_body = Vec3<double>(config.Kd_body_0,
  //  config.Kd_body_1, config.Kd_body_2); userParameters.Kp_foot = Vec3<double>(config.Kp_foot_0,
  //  config.Kp_foot_1, config.Kp_foot_2); userParameters.Kd_foot = Vec3<double>(config.Kd_foot_0,
  //  config.Kd_foot_1, config.Kd_foot_2);

  ROS_INFO_STREAM("New dynamic data!");
}

void Body_Manager::_filterInput() {}
