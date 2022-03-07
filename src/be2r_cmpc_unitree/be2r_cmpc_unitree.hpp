#pragma once

//C++
#include <iostream>

//ROS
#include <ros/ros.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <geometry_msgs/Twist.h>

//MIT
#include "Controllers/StateEstimatorContainer.h"
#include "LegController.h"
#include "MiniCheetah.h"
#include "RobotParameters.h"
// #include "RobotController.h"
#include "Configuration.h"
#include "MIT_Controller.hpp"
#include "spi_command_t.hpp"
#include "spi_data_t.hpp"
#include "spi_torque_t.hpp"

#include <ros/transport_hints.h>

#include "Controllers/ContactEstimator.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"

#define MAIN_LOOP_RATE 500

#define FSM 4

const float max_torque[3] = {17.f, 17.f, 26.f};        // TODO CHECK WITH BEN
const float max_max_torque[3] = {170.f, 170.f, 260.f}; // TODO CHECK WITH BEN
const float wimp_torque[3] = {6.f, 6.f, 6.f};          // TODO CHECK WITH BEN
const float disabled_torque[3] = {0.f, 0.f, 0.f};

class Body_Manager
{
public:
  Body_Manager(RobotController* robot_ctrl);
  ~Body_Manager();

  void init();
  void run();
  void setupStep();
  void finalizeStep();
  void initializeStateEstimator();

  VectorNavData vectorNavData;
  RobotControlParameters controlParameters;
  ControlParameters* _userControlParameters = nullptr;
  SpiData spiData;
  SpiCommand spiCommand;
  u64 _iterations = 0;

  bool is_stand = false;

private:
  ros::NodeHandle _nh;

  ros::Publisher _pub_low_cmd;
  ros::Subscriber _sub_low_state;
  ros::Subscriber _sub_cmd_vel;

  void _lowStateCallback(unitree_legged_msgs::LowState msg);
  void _cmdVelCallback(geometry_msgs::Twist msg);
  void _torqueCalculator(SpiCommand* cmd, SpiData* data, spi_torque_t* torque_out, int board_num);

  Quadruped<float> _quadruped;
  FloatingBaseModel<float> _model;
  LegController<float>* _legController = nullptr;
  StateEstimatorContainer<float>* _stateEstimator;
  StateEstimate<float> _stateEstimate;
  RobotController* _robot_ctrl;
  DesiredStateCommand<float>* _desiredStateCommand;
  GamepadCommand driverCommand;
  rc_control_settings rc_control;

  spi_torque_t _spi_torque;

  void _initSubscribers();
  void _initPublishers();
};
