/*!
 * @file FootSwingTrajectory.h
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#ifndef CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
#define CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H

#include "cppTypes.h"
#include <iostream>
// #include "Controllers/PositionVelocityEstimator.h"
#include "Controllers/StateEstimatorContainer.h"


/*!
 * A foot swing trajectory for a single foot
 */
template<typename T>
class FootSwingTrajectory
{
public:
  /*!
   * Construct a new foot swing trajectory with everything set to zero
   */
  FootSwingTrajectory()
  {
    _p0.setZero();
    _pf.setZero();
    _p.setZero();
    _v.setZero();
    _a.setZero();
    _highestpoint.setZero();
    _mode = 0;
  }

  /*!
   * Set the starting location of the foot
   * @param p0 : the initial foot position
   */
  void setInitialPosition(Vec3<T> p0) { _p0 = p0; }

  /*!
   * Set the starting location of the foot
   * @param stateEstimator stateEstimator  : reference to the estimator data
   */
  void setStateEstimatorAdress(StateEstimatorContainer<T>* stateEstimator) { _stateEstimator = stateEstimator; 
  // std::cout<< "Set State Estimator Adress was completed succesfully!"<< std::endl;
  }

    /*!
   * Set the mode for the basis function: 
   * mode = 0 linear basis function,
   * mode = 1 square root basis function 
   * @param mode : the mode of the Modified interpolate function
   */
  void setMode(int mode) { _mode = mode; }
  const Vec3<T>& getInitialPosition() { return _p0; }

  /*!
   * Set the desired final position of the foot
   * @param pf : the final foot posiiton
   */
  void setFinalPosition(Vec3<T> pf) { _pf = pf; }
  const Vec3<T>& getFinalPosition() { return _pf; }

  /*!
   * Set the maximum height of the swing
   * @param h : the maximum height of the swing, achieved halfway through the
   * swing
   */
  void setHeight(Vec3<T> h) { _highestpoint = h; }

  void computeSwingTrajectoryBezier(T phase, T swingTime, T legside);
  void computeSwingTrajectoryModified(T phase, T swingTime,int mode);
  void computeStairsSwingTrajectoryBezier(T phase, T swingTime);

  /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
   */
  Vec3<T> getPosition() { return _p; }

  /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
   */
  Vec3<T> getVelocity() { return _v; }

  /*!
   * Get the foot acceleration at the current point along the swing
   * @return : the foot acceleration
   */
  Vec3<T> getAcceleration() { return _a; }

private:
  Vec3<T> _p0, _pf, _p, _v, _a;
  StateEstimatorContainer<T>* _stateEstimator = nullptr;
  Vec3<T> _highestpoint;
  int _mode;
};

#endif // CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
