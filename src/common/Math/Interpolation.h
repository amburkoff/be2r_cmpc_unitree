/*! @file Interpolation.h
 *  @brief Utility functions to interpolate between two values
 *
 */

#ifndef PROJECT_INTERPOLATION_H
#define PROJECT_INTERPOLATION_H

#include <assert.h>
#include <type_traits>
#include <cmath>

namespace Interpolate
{

/*!
 * Linear interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t>
y_t lerp(y_t y0, y_t yf, x_t x)
{
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  return y0 + (yf - y0) * x;
}

/*!
 * Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t>
y_t cubicBezier(y_t y0, y_t yf, x_t x)
{
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
  return y0 + bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x)
{
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) * x * (x_t(1) - x);
  return bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x)
{
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) - x_t(12) * x;
  return bezier * yDiff;
}

/*!
 * Cubic bezier modified interpolation between y0 and yf.  t,x is between 0 and 1
 * We took the same path but changed the basis function. It was just proportional to phase of the swing. 
 * Now  we use square root of the phase which output values have the same limits [0,1] but different step. 
 * That will cause to the speed and acceleration profile and dicrease it before the touch.
 */
template <typename y_t, typename x_t>
y_t cubicBezierSQRT(y_t y0, y_t yf, x_t t) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(t >= 0 && t <= 1);
  // t -- is the phase of the swing
  x_t g = sqrt(x_t(1.2)*t+x_t(0.01))-x_t(0.1); //g(t) -- is a new basis function
  y_t yDiff = yf - y0;
  x_t bezier = g * g * g + x_t(3) * (g * g * (x_t(1) - g));//y=f(g(t)) -- Bezie function
  return y0 + bezier * yDiff;
}

/*!
 * Cubic bezier modified interpolation derivative between y0 and yf.  t,x is between 0 and 1
 * We have to differentiate the Bezie function with new basis function relative to the phase t.
 * After that we can compare speed of an ussuall Bezie and our modified version. 
 */
template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivativeSQRT(y_t y0, y_t yf, x_t t) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(t >= 0 && t <= 1);
   // t -- is the phase of the swing
  y_t yDiff = yf - y0;
  x_t g = sqrt(x_t(1.2)*t+x_t(0.01))-x_t(0.1);//g(t) -- is a new basis function
  //dy/dt = df/dg * dg/dt - differentiation rule
  x_t df_dg = x_t(6) * g * (x_t(1) - g);
  x_t dg_dt = x_t(0.6)/sqrt(x_t(1.2)*t+x_t(0.01));
  x_t bezier = df_dg*dg_dt;
  return bezier * yDiff;
}

/*!
 * Cubic bezier modified interpolation derivative between y0 and yf.  t,x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierSecondDerivativeSQRT(y_t y0, y_t yf, x_t t) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(t >= 0 && t <= 1);
   // t -- is the phase of the swing
  y_t yDiff = yf - y0;
  x_t g = sqrt(x_t(1.2)*t+x_t(0.01))-x_t(0.1); //g(t) -- is a new basis function
  //d2y/d2t = d2f/d2g * (dg/dt)^2 + df/dg * d2g/d2t - differentiation rule
  x_t df_dg = x_t(6) * g * (x_t(1) - g);
  x_t dg_dt = x_t(0.6)/sqrt(x_t(1.2)*t+x_t(0.01));
  x_t d2f_d2g = (x_t(6) - x_t(12) * g);
  x_t d2g_d2t = - x_t(0.5)*x_t(0.6)*x_t(1.2)/sqrt((x_t(1.2)*t+x_t(0.01))*(x_t(1.2)*t+x_t(0.01))*(x_t(1.2)*t+x_t(0.01)));
  x_t bezier =  d2f_d2g * dg_dt*dg_dt + df_dg * d2g_d2t;
  return bezier * yDiff;
}

/*!
 * Cosine interpolation between y0 and yf.  t,x is between 0 and 1
 * Previous MIT version was the same as cosine function for the Y-axis when we have a symmetry. 
 * However, when we want to use a new basis function it stops working. 
 * In the maximum hieght with t =0.5(phase) velocities from the left and right region of t are not equal.
 * So we need a new interpolate function:
 *          y = a*cos(w*t + phi) + b
 */
template <typename y_t, typename x_t>
y_t Cosine(y_t y0, y_t yf, y_t h, x_t t) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(t >= 0 && t <= 1);
  static_assert(std::is_floating_point<y_t>::value,
                "must specify input h>0 and yf-y0>=0");
  assert(h > 0 && yf-y0 >= 0);
  x_t g = t;
  x_t pi = x_t(acos(-1));
  x_t phi = pi/x_t(2);
  y_t a = -h/x_t(2);
  y_t b = y0 - a;
  if ((yf-b)/a < 1) {
  x_t w = 2*pi - asin((yf-b)/a) - phi;
  x_t sine = sin(w*g+phi);//y=f(g(t))
  }
  else {
    x_t sine = 0;
  };
  return b + sine* a;
}

} // namespace Interpolate

#endif // PROJECT_INTERPOLATION_H
