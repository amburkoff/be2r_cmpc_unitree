/*! @file Interpolation.h
 *  @brief Utility functions to interpolate between two values
 *
 */

#ifndef PROJECT_INTERPOLATION_H
#define PROJECT_INTERPOLATION_H

#include <assert.h>
#include <type_traits>

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
 */
template <typename y_t, typename x_t>
y_t cubicBezierSQRT(y_t y0, y_t yf, x_t t) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(t >= 0 && t <= 1);
  x_t g = sqrt(x_t(1.2)*t+x_t(0.01))-0.1; //g(t)
  y_t yDiff = yf - y0;
  x_t bezier = g * g * g + x_t(3) * (g * g * (x_t(1) - g));//y=f(g(t))
  return y0 + bezier * yDiff;
}

/*!
 * Cubic bezier modified interpolation derivative between y0 and yf.  t,x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivativeSQRT(y_t y0, y_t yf, x_t t) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(t >= 0 && t <= 1);
  y_t yDiff = yf - y0;
  x_t g = sqrt(x_t(1.2)*t+x_t(0.01))-0.1;//g(t)
  //dy/dt = df/dg * dg/dt
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
  y_t yDiff = yf - y0;
  x_t g = sqrt(x_t(1.2)*t+x_t(0.01))-0.1; //g(t)
  //d2y/d2t = d2f/d2g * (dg/dt)^2 + df/dg * d2g/d2t
  x_t df_dg = x_t(6) * g * (x_t(1) - g);
  x_t dg_dt = x_t(0.6)/sqrt(x_t(1.2)*t+x_t(0.01));
  x_t d2f_d2g = (x_t(6) - x_t(12) * g);
  x_t d2g_d2t = - x_t(0.5)*x_t(0.6)*x_t(1.2)/sqrt((x_t(1.2)*t+x_t(0.01))*(x_t(1.2)*t+x_t(0.01))*(x_t(1.2)*t+x_t(0.01)));
  x_t bezier =  d2f_d2g * dg_dt*dg_dt + df_dg * d2g_d2t;
  return bezier * yDiff;
}

} // namespace Interpolate

#endif // PROJECT_INTERPOLATION_H
