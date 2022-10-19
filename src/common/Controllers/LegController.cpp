/*! @file LegController.cpp
 *  @brief Common Leg Control Interface
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#include "LegController.h"

using namespace std;

/*!
 * Zero the leg command so the leg will not output torque
 */
template<typename T>
void LegControllerCommand<T>::zero()
{
  tauFeedForward = Vec3<T>::Zero();
  forceFeedForward = Vec3<T>::Zero();
  tauSafe = Vec3<T>::Zero();
  qDes = Vec3<T>::Zero();
  qdDes = Vec3<T>::Zero();
  pDes = Vec3<T>::Zero();
  vDes = Vec3<T>::Zero();
  kpCartesian = Mat3<T>::Zero();
  kiCartesian = Mat3<T>::Zero();
  kdCartesian = Mat3<T>::Zero();
  kpJoint = Mat3<T>::Zero();
  kdJoint = Mat3<T>::Zero();
  i_saturation = 0;
}

/*!
 * Zero the leg data
 */
template<typename T>
void LegControllerData<T>::zero()
{
  q = Vec3<T>::Zero();
  qd = Vec3<T>::Zero();
  p = Vec3<T>::Zero();
  v = Vec3<T>::Zero();
  J = Mat3<T>::Zero();
  tauEstimate = Vec3<T>::Zero();
}

/*!
 * Zero all leg commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
template<typename T>
void LegController<T>::zeroCommand()
{
  for (auto& cmd : commands)
  {
    cmd.zero();
  }

  _legEnabled[0] = false;
  _legEnabled[1] = false;
  _legEnabled[2] = false;
  _legEnabled[3] = false;
}

/*!
 * Set the leg to edamp.  This overwrites all command data and generates an
 * emergency damp command using the given gain. For the mini-cheetah, the edamp
 * gain is Nm/(rad/s), and for the Cheetah 3 it is N/m. You still must call
 * updateCommand for this command to end up in the low-level command data!
 */
template<typename T>
void LegController<T>::edampCommand(T gain)
{
  zeroCommand();

  for (int leg = 0; leg < 4; leg++)
  {
    for (int axis = 0; axis < 3; axis++)
    {
      commands[leg].kdJoint(axis, axis) = gain;
    }

    _legEnabled[leg] = true;
  }
}

/*!
 * Update the "leg data" from a SPIne board message
 */
template<typename T>
void LegController<T>::updateData(const SpiData* spiData)
{
  static float fc = 0.8;

  for (int leg = 0; leg < 4; leg++)
  {
    // q:
    datas[leg].q(0) = spiData->q_abad[leg];
    datas[leg].q(1) = spiData->q_hip[leg];
    datas[leg].q(2) = spiData->q_knee[leg];

    // qd
    datas[leg].qd(0) = spiData->qd_abad[leg];
    datas[leg].qd(1) = spiData->qd_hip[leg];
    datas[leg].qd(2) = spiData->qd_knee[leg];

    // J and p
    computeLegJacobianAndPosition<T>(quadruped, datas[leg].q, &(datas[leg].J), &(datas[leg].p), leg);

    // v
    datas[leg].v = datas[leg].J * datas[leg].qd;
  }
}

/*!
 * Update the "leg command" for the SPIne board message
 */
template<typename T>
void LegController<T>::updateCommand(SpiCommand* spiCommand)
{
  for (int leg = 0; leg < 4; leg++)
  {
    // tauFF
    Vec3<T> legTorque = commands[leg].tauFeedForward; // vision, wbc, jposinit and control, invdyn,

    // forceFF
    Vec3<T> footForce = commands[leg].forceFeedForward;

    commands[leg].integral += (commands[leg].pDes - datas[leg].p);

    for (size_t i = 0; i < 3; i++)
    {
      if (commands[leg].integral(i) > commands[leg].i_saturation)
      {
        commands[leg].integral(i) = commands[leg].i_saturation;
      }
      else if (commands[leg].integral(i) < -commands[leg].i_saturation)
      {
        commands[leg].integral(i) = -commands[leg].i_saturation;
      }
    }

    // cartesian PID
    footForce += commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p);
    footForce += commands[leg].kiCartesian * commands[leg].integral;
    footForce += commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v);

    // Torque
    legTorque += datas[leg].J.transpose() * footForce;
    // legTorque += commands[leg].tauSafe;

    // set command:
    spiCommand->tau_abad_ff[leg] = legTorque(0);
    spiCommand->tau_hip_ff[leg] = legTorque(1);
    spiCommand->tau_knee_ff[leg] = legTorque(2);

    // joint space P
    spiCommand->kp_abad[leg] = commands[leg].kpJoint(0, 0);
    spiCommand->kp_hip[leg] = commands[leg].kpJoint(1, 1);
    spiCommand->kp_knee[leg] = commands[leg].kpJoint(2, 2);

    // joint space D
    spiCommand->kd_abad[leg] = commands[leg].kdJoint(0, 0);
    spiCommand->kd_hip[leg] = commands[leg].kdJoint(1, 1);
    spiCommand->kd_knee[leg] = commands[leg].kdJoint(2, 2);

    // is low level control -> change signs
    if (is_low_level)
    {
      spiCommand->q_des_abad[leg] = commands[leg].qDes(0);
      spiCommand->q_des_hip[leg] = -commands[leg].qDes(1);
      spiCommand->q_des_knee[leg] = -commands[leg].qDes(2);

      spiCommand->qd_des_abad[leg] = commands[leg].qdDes(0);
      spiCommand->qd_des_hip[leg] = -commands[leg].qdDes(1);
      spiCommand->qd_des_knee[leg] = -commands[leg].qdDes(2);
    }
    else
    {
      spiCommand->q_des_abad[leg] = commands[leg].qDes(0);
      spiCommand->q_des_hip[leg] = commands[leg].qDes(1);
      spiCommand->q_des_knee[leg] = commands[leg].qDes(2);

      spiCommand->qd_des_abad[leg] = commands[leg].qdDes(0);
      spiCommand->qd_des_hip[leg] = commands[leg].qdDes(1);
      spiCommand->qd_des_knee[leg] = commands[leg].qdDes(2);
    }

    // estimate torque
    datas[leg].tauEstimate = legTorque + commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +
                             commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);

    spiCommand->flags[leg] = _legEnabled[leg];

    // if (leg == 0)
    // {
    //   cout << "tau ff: " << commands[0].tauFeedForward << endl;
    //   cout << "f ff: " << commands[0].forceFeedForward << endl;
    //   cout << "f imp: " << footForce << endl;
    //   cout << "Kp: " << commands[0].kpCartesian << endl;
    //   cout << "Kd: " << commands[0].kdCartesian << endl;
    //   cout << "Kp abad: " << spiCommand->kp_abad[0] << endl;
    //   cout << "Kp hip: " << spiCommand->kp_hip[0] << endl;
    //   cout << "Kp knee: " << spiCommand->kp_knee[0] << endl;
    //   cout << "Kd abad: " << spiCommand->kd_abad[0] << endl;
    //   cout << "Kd hip: " << spiCommand->kd_hip[0] << endl;
    //   cout << "Kd knee: " << spiCommand->kd_knee[0] << endl;
    // }
  }
}

template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;

template struct LegControllerData<double>;
template struct LegControllerData<float>;

template class LegController<double>;
template class LegController<float>;

/*!
 * Compute the position of the foot and its Jacobian.  This is done in the local
 * leg coordinate system. If J/p are NULL, the calculation will be skipped.
 */
template<typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J, Vec3<T>* p, int leg)
{
  T l1 = quad._abadLinkLength;
  T l2 = quad._hipLinkLength;
  T l3 = quad._kneeLinkLength;
  T l4 = quad._kneeLinkY_offset;
  T sideSign = quad.getSideSign(leg);

  T s1 = std::sin(q(0));
  T s2 = std::sin(q(1));
  T s3 = std::sin(q(2));

  T c1 = std::cos(q(0));
  T c2 = std::cos(q(1));
  T c3 = std::cos(q(2));

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;

  if (J)
  {
    J->operator()(0, 0) = 0;
    J->operator()(0, 1) = l3 * c23 + l2 * c2;
    J->operator()(0, 2) = l3 * c23;
    J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1;
    J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) = l3 * c1 * s23;
  }

  if (p)
  {
    p->operator()(0) = l3 * s23 + l2 * s2;
    p->operator()(1) = (l1 + l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = (l1 + l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

template void computeLegJacobianAndPosition<double>(Quadruped<double>& quad,
                                                    Vec3<double>& q,
                                                    Mat3<double>* J,
                                                    Vec3<double>* p,
                                                    int leg);

template void computeLegJacobianAndPosition<float>(Quadruped<float>& quad,
                                                   Vec3<float>& q,
                                                   Mat3<float>* J,
                                                   Vec3<float>* p,
                                                   int leg);
