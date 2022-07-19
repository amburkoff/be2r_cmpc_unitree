/*! @file PositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

//#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
//#define PROJECT_POSITIONVELOCITYESTIMATOR_H
#pragma once
#include "Controllers/StateEstimatorContainer.h"


/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */
template <typename T>
class KFPositionVelocityEstimator : public GenericEstimator<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KFPositionVelocityEstimator();
  virtual void run();
  virtual void setup();

private:
  Eigen::Matrix<T, 18, 1> _xhat;
  Eigen::Matrix<T, 18, 1> _myxhat;
  Eigen::Matrix<T, 12, 1> _ps;
  Eigen::Matrix<T, 12, 1> _vs;
  Eigen::Matrix<T, 18, 18> _A;
  Eigen::Matrix<T, 21, 21> _Ad;
  Eigen::Matrix<T, 21, 21> _dAd;
  Eigen::Matrix<T, 18, 18> _Q0;
  Eigen::Matrix<T, 18, 18> _P;
  Eigen::Matrix<T, 28, 28> _R0;
  Eigen::Matrix<T, 18, 3> _B;
  Eigen::Matrix<T, 28, 18> _C;
  Eigen::Matrix<T, 2, 2> _Af;
  Eigen::Matrix<T, 2, 2> _Afd;
  Eigen::Matrix<T, 2, 1> _Bf;
  Eigen::Matrix<T, 2, 1> _Bfd;
  Eigen::Matrix<T, 1, 2> _Cf;
  Vec3<T> a_old;
  Vec3<T> a_filt;
  Vec3<T> da_filt;
  Vec3<T> da_filt_prev;
  Vec3<T> _a_world;
  Vec2<T> _a_worldx;
  Vec2<T> _a_worldy;
  Vec2<T> _a_worldz;
  T _w;
};

/*!
 * "Cheater" position and velocity estimator which will return the correct position and
 * velocity when running in simulation.
 */
// template <typename T>
// class CheaterPositionVelocityEstimator : public GenericEstimator<T>
// {
// public:
//   virtual void run();
//   virtual void setup() {}
// };

//#endif // PROJECT_POSITIONVELOCITYESTIMATOR_H
