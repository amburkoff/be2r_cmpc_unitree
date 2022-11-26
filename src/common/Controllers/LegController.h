/*! @file LegController.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards (the low level leg control boards)
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#pragma once

#include "Dynamics/Quadruped.h"
#include "cppTypes.h"
#include <iostream>
#include <ros/ros.h>

/*!
 * Command to spine board
 */
struct SpiCommand
{
  float q_des_abad[4];
  float q_des_hip[4];
  float q_des_knee[4];

  float qd_des_abad[4];
  float qd_des_hip[4];
  float qd_des_knee[4];

  float kp_abad[4];
  float kp_hip[4];
  float kp_knee[4];

  float kd_abad[4];
  float kd_hip[4];
  float kd_knee[4];

  float tau_abad_ff[4];
  float tau_hip_ff[4];
  float tau_knee_ff[4];

  float tau_abad_safe[4];
  float tau_hip_safe[4];
  float tau_knee_safe[4];

  int32_t flags[4];
};

/*!
 * Data from spine board
 */
struct SpiData
{
  float q_abad[4];
  float q_hip[4];
  float q_knee[4];
  float qd_abad[4];
  float qd_hip[4];
  float qd_knee[4];
  int32_t flags[4];
  int32_t spi_driver_status;
};

/*!
 * Data sent from the control algorithm to the legs.
 */
template<typename T>
struct LegControllerCommand
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand()
  {
    zero();
  }

  void zero();

  Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes, integral, tauSafe;
  Mat3<T> kpCartesian, kdCartesian, kiCartesian, kpJoint, kdJoint;
  float i_saturation;
};

/*!
 * Data returned from the legs to the control code.
 */
template<typename T>
struct LegControllerData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData()
  {
    zero();
  }

  void setQuadruped(Quadruped<T>& quad)
  {
    quadruped = &quad;
  }

  void zero();

  Vec3<T> q, qd, p, v;
  Mat3<T> J;
  Vec3<T> tauEstimate;
  Quadruped<T>* quadruped;
};

/*!
 * Controller for 4 legs of a quadruped.  Works for both Mini Cheetah and Cheetah 3
 */
template<typename T>
class LegController
{
public:
  LegController(Quadruped<T>& quad)
    : quadruped(quad)
  {
    for (auto& data : datas)
      data.setQuadruped(quadruped);
  }

  void zeroCommand();
  void edampCommand(T gain);
  void updateData(const SpiData* spiData);
  void updateCommand(SpiCommand* spiCommand);
  void setEnabled(bool enabled)
  {
    _legEnabled[0] = enabled;
    _legEnabled[1] = enabled;
    _legEnabled[2] = enabled;
    _legEnabled[3] = enabled;
  };

  void setLegEnabled(uint8_t leg_num, bool state)
  {
    _legEnabled[leg_num] = state;
  }

  LegControllerCommand<T> commands[4];
  LegControllerData<T> datas[4];
  Quadruped<T>& quadruped;
  bool _legEnabled[4] = { false };
  T _maxTorque = 0;
  bool _zeroEncoders = false;
  u32 _calibrateEncoders = 0;
  bool is_low_level = false;
};

template<typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J, Vec3<T>* p, int leg);
