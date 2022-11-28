/*!
 * @file FootSwingTrajectory.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#include "Controllers/FootSwingTrajectory.h"
#include "Math/Interpolation.h"

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */
template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryBezier(T phase, T swingTime, T legside)
{
  // _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);
  // _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase) / swingTime;
  // _a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _pf, phase) / (swingTime * swingTime);

  // T zp, zv, za;
  // T yp, yv, ya;
  Vec3<T> _shifted_extr= this->_stateEstimator->getResult().rBody.transpose()* Vec3<T>(_highestpoint[0],legside*_highestpoint[1],_highestpoint[3]);
  if (phase < T(0.5))
  {
    // zp = Interpolate::cubicBezier<T>(_p0[2], _p0[2] + _shifted_extr[3] phase * 2);
    // zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2], _p0[2] + _shifted_extr[3], phase * 2) * 2 / swingTime;
    // za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2], _p0[2] + _shifted_extr[3], phase * 2) * 4 / (swingTime * swingTime);
    _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _p0 + _shifted_extr, phase * 2);
    _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _p0 + _shifted_extr, phase * 2) * 2 / swingTime;
    _a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _p0 + _shifted_extr, phase * 2) * 4 / (swingTime * swingTime);
  }
  else
  {
    // zp = Interpolate::cubicBezier<T>(_p0[2] + _shifted_extr[3], _pf[2], phase * 2 - 1);
    // zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2] + _shifted_extr[3], _pf[2], phase * 2 - 1) * 2 / swingTime;
    // za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2] + _shifted_extr[3], _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
    _p = Interpolate::cubicBezier<Vec3<T>>(_p0 + _shifted_extr, _pf, phase * 2 - 1);
    _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0+ _shifted_extr, _pf, phase * 2 - 1) * 2 / swingTime;
    _a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0 + legside*_shifted_extr, _pf, phase * 2 - 1) * 4 / (swingTime * swingTime);
 }

  // _p[2] = zp;
  // _v[2] = zv;
  // _a[2] = za;
  // _p[1] = yp;
  // _v[1] = yv;
  // _a[1] = ya;
}

template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryModified(T phase, T swingTime, int mode)
{
  T zp, zv, za;

  if (mode == 0)
  {
    _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);
    _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase) / swingTime;
    _a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _pf, phase) / (swingTime * swingTime);
    zp = Interpolate::ShiftedSine<T>(_p0[2],  _pf[2], _highestpoint[3], phase);
    zv = Interpolate::ShiftedSineFirstDerivative<T>(_p0[2],  _pf[2], _highestpoint[3], phase) /swingTime;
    za = Interpolate::ShiftedSineSecondDerivative<T>(_p0[2],  _pf[2], _highestpoint[3], phase)/ (swingTime * swingTime);

  _p[2] = zp;
  _v[2] = zv;
  _a[2] = za;
  }
  else{
    _p = Interpolate::cubicBezierSQRT<Vec3<T>>(_p0, _pf, phase);
    _v = Interpolate::cubicBezierFirstDerivativeSQRT<Vec3<T>>(_p0, _pf, phase) / swingTime;
    _a = Interpolate::cubicBezierSecondDerivativeSQRT<Vec3<T>>(_p0, _pf, phase) / (swingTime * swingTime);

    zp = Interpolate::ShiftedSineSQRT<T>(_p0[2],  _pf[2], _highestpoint[3], phase);
    zv = Interpolate::ShiftedSineFirstDerivativeSQRT<T>(_p0[2],  _pf[2], _highestpoint[3], phase) /swingTime;
    za = Interpolate::ShiftedSineSecondDerivativeSQRT<T>(_p0[2],  _pf[2], _highestpoint[3], phase)/ (swingTime * swingTime);

    _p[2] = zp;
    _v[2] = zv;
    _a[2] = za;
  }
}

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */
template <typename T>
void FootSwingTrajectory<T>::computeStairsSwingTrajectoryBezier(T phase, T swingTime)
{
  _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);
  _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase) / swingTime;
  _a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _pf, phase) / (swingTime * swingTime);

  T zp, zv, za;
  T xp, xv, xa;
  T yp, yv, ya;

  if (phase < T(0.5))
  {
    zp = Interpolate::cubicBezier<T>(_p0[2], _p0[2] + _highestpoint[3], phase * 2);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2], _p0[2] + _highestpoint[3], phase * 2) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2], _p0[2] + _highestpoint[3], phase * 2) * 4 / (swingTime * swingTime);

    xp = Interpolate::cubicBezier<T>(_p0[0], _p0[0], phase * 2);
    xv = Interpolate::cubicBezierFirstDerivative<T>(_p0[0], _p0[0] + (_pf[0] - _p0[0]) / 3, phase * 2) * 2 / swingTime;
    xa = Interpolate::cubicBezierSecondDerivative<T>(_p0[0], _p0[0] + (_pf[0] - _p0[0]) / 3, phase * 2) * 4 / (swingTime * swingTime);

    yp = Interpolate::cubicBezier<T>(_p0[1], _p0[1], phase * 2);
    yv = Interpolate::cubicBezierFirstDerivative<T>(_p0[1], _p0[1] + (_pf[1] - _p0[1]) / 3, phase * 2) * 2 / swingTime;
    ya = Interpolate::cubicBezierSecondDerivative<T>(_p0[1], _p0[1] + (_pf[1] - _p0[1]) / 3, phase * 2) * 4 / (swingTime * swingTime);
  }
  else
  {
    zp = Interpolate::cubicBezier<T>(_p0[2] + _highestpoint[3], _pf[2], phase * 2 - 1);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2] + _highestpoint[3], _pf[2], phase * 2 - 1) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2] + _highestpoint[3], _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);

    xp = Interpolate::cubicBezier<T>(_p0[0], _p0[0], phase * 2 - 1);
    xv = Interpolate::cubicBezierFirstDerivative<T>(_p0[0] + (_pf[0] - _p0[0]) / 3, _pf[0], phase * 2 - 1) * 2 / swingTime;
    xa = Interpolate::cubicBezierSecondDerivative<T>(_p0[0] + (_pf[0] - _p0[0]) / 3, _pf[0], phase * 2 - 1) * 4 / (swingTime * swingTime);

    yp = Interpolate::cubicBezier<T>(_p0[1], _p0[1], phase * 2 - 1);
    yv = Interpolate::cubicBezierFirstDerivative<T>(_p0[1] + (_pf[1] - _p0[1]) / 3, _pf[1], phase * 2 - 1) * 2 / swingTime;
    ya = Interpolate::cubicBezierSecondDerivative<T>(_p0[1] + (_pf[1] - _p0[1]) / 3, _pf[1], phase * 2 - 1) * 4 / (swingTime * swingTime);
  }

  _p[0] = xp;
  _v[0] = xv;
  _a[0] = xa;
  _p[1] = yp;
  _v[1] = yv;
  _a[1] = ya;
  _p[2] = zp;
  _v[2] = zv;
  _a[2] = za;
}

template class FootSwingTrajectory<float>;
