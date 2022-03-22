#pragma once

//C++
#include <iostream>

//ROS
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>

//MIT
#include "Configuration.h"
#include "ControlFSM.h"
#include "Controllers/StateEstimatorContainer.h"
#include "LegController.h"
#include "MiniCheetah.h"
#include "RobotParameters.h"
#include "spi_command_t.hpp"
#include "spi_data_t.hpp"
#include "spi_torque_t.hpp"
// #include "Controllers/ContactEstimator.h"
#include "Controllers/GaitScheduler.h"

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
  Body_Manager();
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
  Vec4<uint8_t> footContactState;

  bool is_stand = false;

private:
  ros::NodeHandle _nh;

  ros::Publisher _pub_low_cmd;
  ros::Subscriber _sub_low_state;
  ros::Subscriber _sub_cmd_vel;

  void _initSubscribers();
  void _initPublishers();
  void _filterInput();

  void _lowStateCallback(unitree_legged_msgs::LowState msg);
  void _cmdVelCallback(geometry_msgs::Twist msg);
  void _torqueCalculator(SpiCommand* cmd, SpiData* data, spi_torque_t* torque_out, int board_num);

  Quadruped<float> _quadruped;
  FloatingBaseModel<float> _model;
  LegController<float>* _legController = nullptr;
  StateEstimatorContainer<float>* _stateEstimator;
  StateEstimate<float> _stateEstimate;
  DesiredStateCommand<float>* _desiredStateCommand;
  GamepadCommand driverCommand;

  spi_torque_t _spi_torque;

  ControlFSM<float>* _controlFSM;

  // Gait Scheduler controls the nominal contact schedule for the feet
  GaitScheduler<float>* _gaitScheduler;
  MIT_UserParameters userParameters;

};
