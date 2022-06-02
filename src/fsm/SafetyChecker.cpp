/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 */

#include "SafetyChecker.h"
#include "iostream"

using namespace std;

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkSafeOrientation()
{
  // cout << "[SafetyChecker] checkSafeOrientation func start" << endl;

  if (abs(data->_stateEstimator->getResult().rpy(0)) >= 1.0 ||
      abs(data->_stateEstimator->getResult().rpy(1)) >= 1.0)
  {
    cout << "[SafetyChecker] Roll is " << abs(data->_stateEstimator->getResult().rpy(0)) << endl;
    cout << "[SafetyChecker] Pitch is " << abs(data->_stateEstimator->getResult().rpy(1)) << endl;

    printf("Orientation safety check failed!\n");
    return false;
  }
  else
  {
    return true;
  }
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkPDesFoot()
{
  // Assumed safe to start
  bool safePDesFoot = true;

  // Safety parameters
  T maxAngle = 1.0472; // 60 degrees (should be changed)
  T maxPDes = data->_quadruped->_maxLegLength * sin(maxAngle);

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++)
  {
    // Keep the foot from going too far from the body in +x
    if (data->_legController->commands[leg].pDes(0) > maxPDes)
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(0)
                << " | modified: " << maxPDes << std::endl;
      data->_legController->commands[leg].pDes(0) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -x
    if (data->_legController->commands[leg].pDes(0) < -maxPDes)
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(0)
                << " | modified: " << -maxPDes << std::endl;
      data->_legController->commands[leg].pDes(0) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in +y
    if (data->_legController->commands[leg].pDes(1) > maxPDes)
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(1)
                << " | modified: " << maxPDes << std::endl;
      data->_legController->commands[leg].pDes(1) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -y
    if (data->_legController->commands[leg].pDes(1) < -maxPDes)
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(1)
                << " | modified: " << -maxPDes << std::endl;
      data->_legController->commands[leg].pDes(1) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the leg under the motor module (don't raise above body or crash into
    // module)
    if (data->_legController->commands[leg].pDes(2) >
        -data->_quadruped->_maxLegLength / 4)
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(2)
                << " | modified: " << -data->_quadruped->_maxLegLength / 4
                << std::endl;
      data->_legController->commands[leg].pDes(2) =
          -data->_quadruped->_maxLegLength / 4;
      safePDesFoot = false;
    }

    // Keep the foot within the kinematic limits
    if (data->_legController->commands[leg].pDes(2) <
        -data->_quadruped->_maxLegLength)
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(2)
                << " | modified: " << -data->_quadruped->_maxLegLength
                << std::endl;
      data->_legController->commands[leg].pDes(2) =
          -data->_quadruped->_maxLegLength;
      safePDesFoot = false;
    }
  }

  // Return true if all desired positions are safe
  return safePDesFoot;
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::checkForceFeedForward()
{
  // Assumed safe to start
  bool safeForceFeedForward = true;

  // Initialize maximum vertical and lateral forces
  T maxLateralForce = 0;
  T maxVerticalForce = 0;

  // Maximum force limits for each robot
  if (data->_quadruped->_robotType == RobotType::CHEETAH_3)
  {
    maxLateralForce = 1800;
    maxVerticalForce = 1800;
  }
  else if (data->_quadruped->_robotType == RobotType::MINI_CHEETAH)
  {
    maxLateralForce = 350;
    maxVerticalForce = 350;
  }

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++)
  {
    // Limit the lateral forces in +x body frame
    if (data->_legController->commands[leg].forceFeedForward(0) >
        maxLateralForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(0)
                << " | modified: " << maxLateralForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(0) = maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -x body frame
    if (data->_legController->commands[leg].forceFeedForward(0) <
        -maxLateralForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(0)
                << " | modified: " << -maxLateralForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(0) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in +y body frame
    if (data->_legController->commands[leg].forceFeedForward(1) >
        maxLateralForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(1)
                << " | modified: " << maxLateralForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(1) = maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -y body frame
    if (data->_legController->commands[leg].forceFeedForward(1) <
        -maxLateralForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(1)
                << " | modified: " << -maxLateralForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(1) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in +z body frame
    if (data->_legController->commands[leg].forceFeedForward(2) >
        maxVerticalForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(2)
                << " | modified: " << -maxVerticalForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(2) =
          maxVerticalForce;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in -z body frame
    if (data->_legController->commands[leg].forceFeedForward(2) <
        -maxVerticalForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(2)
                << " | modified: " << maxVerticalForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(2) =
          -maxVerticalForce;
      safeForceFeedForward = false;
    }
  }

  // Return true if all feed forward forces are safe
  return safeForceFeedForward;
}

template <typename T>
bool SafetyChecker<T>::checkJointLimits()
{
  //from software guide unitree a1
  //hip -46 ~ 46 deg (-0.8 ~ 0.8 rad)
  //thigh -60 ~ 240 deg (-1.04 ~ 4.18 rad)
  //calf -154.5 ~ -52.5 deg (-2.68 ~ -0.907 rad)
  //from urdf
  // lower="-0.802851455917" upper="0.802851455917" hip
  // lower="-1.0471975512" upper="4.18879020479" thigh
  // lower="-2.69653369433" upper="-0.916297857297" calf

  //real
  //hip -0.876 ~ 0.896 rad
  //thigh -1.092 ~ 4.142 rad
  //calf -2.677 ~ -0.854 rad

  //changed sighs for MiniCheetah
  static const float limit_joint0[2] = {-46 * M_PI / 180, 46 * M_PI / 180};
  static const float limit_joint1[2] = {-240 * M_PI / 180, 60 * M_PI / 180};
  static const float limit_joint2[2] = {52.5 * M_PI / 180, 154.5 * M_PI / 180};

  static const float safety_spread = 1.0;

  for (size_t i = 0; i < 4; i++)
  {
    //joint 0 min
    if (data->_legController->datas[i].q(0) < limit_joint0[0] * safety_spread)
    {
      ROS_ERROR_STREAM_ONCE("Leg: " << i << " joint: 0 min limit exceeded! Act: " << data->_legController->datas[i].q(0) << " Limit: " << limit_joint0[0] * safety_spread);
      return false;
    }
    //joint 0 max
    if (data->_legController->datas[i].q(0) > limit_joint0[1] * safety_spread)
    {
      ROS_ERROR_STREAM_ONCE("Leg: " << i << " joint: 0 max limit exceeded! Act: " << data->_legController->datas[i].q(0) << " Limit: " << limit_joint0[1] * safety_spread);
      return false;
    }

    //joint 1 min
    if (data->_legController->datas[i].q(1) < limit_joint1[0] * safety_spread)
    {
      ROS_ERROR_STREAM_ONCE("Leg: " << i << " joint: 1 min limit exceeded! Act: " << data->_legController->datas[i].q(1) << " Limit: " << limit_joint1[0] * safety_spread);
      return false;
    }
    //joint 1 max
    if (data->_legController->datas[i].q(1) > limit_joint1[1] * safety_spread)
    {
      ROS_ERROR_STREAM_ONCE("Leg: " << i << " joint: 1 max limit exceeded! Act: " << data->_legController->datas[i].q(1) << " Limit: " << limit_joint1[1] * safety_spread);
      return false;
    }

    //joint 2 min
    if (data->_legController->datas[i].q(2) < limit_joint2[0] * safety_spread)
    {
      ROS_ERROR_STREAM_ONCE("Leg: " << i << " joint: 2 min limit exceeded! Act: " << data->_legController->datas[i].q(2) << " Limit: " << limit_joint2[0] * safety_spread);
      return false;
    }
    //joint 2 max
    if (data->_legController->datas[i].q(2) > limit_joint2[1] * safety_spread)
    {
      ROS_ERROR_STREAM_ONCE("Leg: " << i << " joint: 2 max limit exceeded! Act: " << data->_legController->datas[i].q(2) << " Limit: " << limit_joint2[1] * safety_spread);
      return false;
    }
  }

  return true;
}

// template class SafetyChecker<double>; This should be fixed... need to make
// RobotRunner a template
template class SafetyChecker<float>;
