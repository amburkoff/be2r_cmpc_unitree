#pragma once

// C++
#include <iostream>

// ROS
#include <Utilities/ros_read_param.h>
#include <be2r_cmpc_unitree/ros_dynamic_paramsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <unitree_legged_msgs/LegError.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/StateError.h>

// MIT
#include "Configuration.h"
#include "ControlFSM.h"
#include "Controllers/StateEstimatorContainer.h"
#include "LegController.h"
#include "MiniCheetah.h"
#include "RobotParameters.h"
#include "lcm_msgs/spi_command_t.hpp"
#include "lcm_msgs/spi_data_t.hpp"
#include "lcm_msgs/spi_torque_t.hpp"
// #include "Controllers/ContactEstimator.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"

// BE2R
#include "debug.hpp"

#define MAIN_LOOP_RATE 500

// #define FSM 3
// #define FSM_AUTO

#define MOTOR_BREAK 0x00
#define MOTOR_ON 0x0A

const float max_torque[3] = { 17.f, 17.f, 26.f };        // TODO CHECK WITH BEN
const float max_max_torque[3] = { 170.f, 170.f, 260.f }; // TODO CHECK WITH BEN
const float wimp_torque[3] = { 6.f, 6.f, 6.f };          // TODO CHECK WITH BEN
const float disabled_torque[3] = { 0.f, 0.f, 0.f };

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

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
  ros::Publisher _pub_joint_states;
  ros::Publisher _pub_state_error;
  ros::Publisher _pub_leg_error;
  ros::Time _time_start;
  const ros::Time _zero_time;
  bool _is_param_updated = false;

  dynamic_reconfigure::Server<be2r_cmpc_unitree::ros_dynamic_paramsConfig> server;
  dynamic_reconfigure::Server<be2r_cmpc_unitree::ros_dynamic_paramsConfig>::CallbackType f;

  void _initSubscribers();
  void _initPublishers();
  void _filterInput();
  void _initParameters();
  void _updateVisualization();
  void _updatePlot();
  void _tfPublish();

  void _lowStateCallback(unitree_legged_msgs::LowState msg);
  void _cmdVelCallback(geometry_msgs::Twist msg);
  void _torqueCalculator(SpiCommand* cmd, SpiData* data, spi_torque_t* torque_out, int board_num);
  void _callbackDynamicROSParam(be2r_cmpc_unitree::ros_dynamic_paramsConfig& config,
                                uint32_t level);

  Quadruped<float> _quadruped;
  FloatingBaseModel<float> _model;
  LegController<float>* _legController = nullptr;
  LegControllerCommand<float> _leg_contoller_params[4];
  StateEstimatorContainer<float>* _stateEstimator;
  StateEstimate<float> _stateEstimate;
  DesiredStateCommand<float>* _desiredStateCommand;
  GamepadCommand driverCommand;
  unitree_legged_msgs::LowState _low_state;
  tf::TransformBroadcaster odom_broadcaster;

  spi_torque_t _spi_torque;

  ControlFSM<float>* _controlFSM;

  // Gait Scheduler controls the nominal contact schedule for the feet
  GaitScheduler<float>* _gaitScheduler;
  ControlParameters* _userControlParameters = nullptr;
  be2r_cmpc_unitree::ros_dynamic_paramsConfig _rosParameters;

  template<typename T>
  bool readRosParam(std::string param_name, T& param_var)
  {
    if (!_nh.getParam(param_name, param_var))
    {
      ROS_WARN_STREAM("Can't read param " << param_name);
      return false;
    }
    // std::cout << "[ROS PARAM] " << param_name << ": " << param_var << std::endl;
    return true;
  }
};
