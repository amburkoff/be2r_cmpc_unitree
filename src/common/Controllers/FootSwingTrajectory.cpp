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
  Vec3<float> _flag(0,1,1);
  // T yp, yv, ya;*
  T yaw = T(0);

  if (_stateEstimator != nullptr){
    
    auto seR = this->_stateEstimator->getResult();
    yaw = seR.rpy[2];
    // std::cout << " :"<< yaw << ": " << endl;
  }
   
  Mat3<T> R = coordinateRotation(CoordinateAxis::Z,yaw);
  //  R.transpose()*
  // std::cout << this->_stateEstimator->getResult().rpy[2] << " " << std::endl;
  Vec3<T> _shifted_extr= Vec3<T>(_highestpoint[0],legside*_highestpoint[1],_highestpoint[2]);
  Vec3<T> _pf_shift = R*(_pf - _p0);
  // std::cout << _shifted_extr[0]<< " "<< _shifted_extr[1]<< " "<< _shifted_extr[2]<<" yaw: " << yaw << std::endl;
  for (int i = 0; i < 3; i++)
  {
    if (_flag[i]==1)
    {
      if (phase < T(0.5))
      {
        // zp = Interpolate::cubicBezier<T>(_p0[2], _p0[2] + _shifted_extr[3] phase * 2);
        // zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2], _p0[2] + _shifted_extr[3], phase * 2) * 2 / swingTime;
        // za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2], _p0[2] + _shifted_extr[3], phase * 2) * 4 / (swingTime * swingTime);
        _p[i] = Interpolate::cubicBezier<T>(T(0), _shifted_extr[i], phase * 2);
        _v[i] = Interpolate::cubicBezierFirstDerivative<T>(T(0), _shifted_extr[i], phase * 2) * 2 / swingTime;
        _a[i] = Interpolate::cubicBezierSecondDerivative<T>(T(0), _shifted_extr[i], phase * 2) * 4 / (swingTime * swingTime);
      }
      else
      {
        // zp = Interpolate::cubicBezier<T>(_p0[2] + _shifted_extr[3], _pf[2], phase * 2 - 1);
        // zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2] + _shifted_extr[3], _pf[2], phase * 2 - 1) * 2 / swingTime;
        // za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2] + _shifted_extr[3], _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
        _p[i] = Interpolate::cubicBezier<T>( _shifted_extr[i], _pf_shift[i], phase * 2 - 1);
        _v[i] = Interpolate::cubicBezierFirstDerivative<T>( _shifted_extr[i], _pf_shift[i], phase * 2 - 1) * 2 / swingTime;
        _a[i] = Interpolate::cubicBezierSecondDerivative<T>(_shifted_extr[i], _pf_shift[i], phase * 2 - 1) * 4 / (swingTime * swingTime);
      }
    }
    else
    {
      _p[i] = Interpolate::cubicBezier<T>(T(0), _pf_shift[i], phase);
      _v[i] = Interpolate::cubicBezierFirstDerivative<T>(T(0), _pf_shift[i], phase) / swingTime;
      _a[i] = Interpolate::cubicBezierSecondDerivative<T>(T(0), _pf_shift[i], phase) / (swingTime * swingTime);
    }
  }
  
  // T zp, zv, za;
  _p = _p0 + R.transpose()*_p;
  _v = R.transpose()*_v;
  _a = R.transpose()*_a;
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
