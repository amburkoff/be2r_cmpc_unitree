#pragma once

#include "Controllers/StateEstimatorContainer.h"
#include <nav_msgs/Path.h>
#include <ros/ros.h>

struct Reference_Trajectory
{
  std::vector<Eigen::Matrix<float, 12, 1>> x_des;
  nav_msgs::Path x_des_ros;
};

class RTG
{
public:
  RTG(LegControllerData<float>* leg_controller_data, StateEstimate<float>* state_estimate);
  ~RTG();

  void setHorizon(uint16_t horizon);
  void generateTrajectory();
  void convertTrajectoryToRosPath();

private:
  Reference_Trajectory _rt;
  LegControllerData<float>* _leg_controller_data = nullptr;
  StateEstimate<float>* _state_estimate = nullptr;
  uint16_t _horizon = 10;
  float _dt = 0.002;
};
