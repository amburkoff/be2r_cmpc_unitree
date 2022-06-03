#include "Controllers/be2rPositionVelocityEstimator.h"

using std::cout;
using std::endl;

template <typename T>
PositionEstimator<T>::PositionEstimator()
{
  time_start = ros::Time::now();
  acc_buffer.set_capacity(MOVING_AVERAGE);
  vel_buffer.set_capacity(MOVING_AVERAGE);

  for (size_t i = 0; i < MOVING_AVERAGE; i++)
  {
    acc_buffer.push_back(Vec3<T>(0, 0, 0));
    vel_buffer.push_back(Vec3<T>(0, 0, 0));
  }
}

/*!
 * Initialize the state estimator
 */
template <typename T>
void PositionEstimator<T>::setup()
{
  a_filtered << 0, 0, 0;
  a_filtered2 << 0, 0, 0;
  p_body << 0, 0, 0.056;
  v_body << 0, 0, 0;
}

/*!
 * Run state estimator
 */
template <typename T>
void PositionEstimator<T>::run()
{
  T dt = 0.002;
  float filter = 0.5;
  static uint16_t counter = 0;
  static float z = 0.056;
  static Vec3<float> vz_filtered(0, 0, 0);
  static Vec3<float> vz_filtered2(0, 0, 0);
  // float phase[4];

  Vec3<T> acceleration = this->_stateEstimatorData.result->aWorld + Vec3<float>(0, 0, -9.81);
  
  // phase[0] = this->_stateEstimatorData.result->contactEstimate(0);
  // phase[1] = this->_stateEstimatorData.result->contactEstimate(1);
  // phase[2] = this->_stateEstimatorData.result->contactEstimate(2);
  // phase[3] = this->_stateEstimatorData.result->contactEstimate(3);

  // float trust_window = 0.1;
  // float trust = 0;

  // for (size_t i = 0; i < 4; i++)
  // {
  //   if (phase[i] < trust_window)
  //   {
  //     trust += phase[i] / trust_window / 5;
  //   }
  //   else
  //   {
  //     trust += 0.2;
  //   }
  // }

  if (counter <= 500)
  {
    counter++;
  }
  else
  {
    // filter = trust;
    a_filtered = _filter(acceleration, 1);
    a_filtered2 = a_filtered2 * (1 - filter) + a_filtered * filter;

    vz_filtered = _filter(Vec3<float>(0, 0, this->_stateEstimatorData.result->vWorld[2]), 0);
    vz_filtered2 = vz_filtered2 * (1 - filter) + vz_filtered * filter;

    // v_body += acceleration * dt;
    v_body += a_filtered2 * dt;
    // v_body += a_filtered * dt;
    p_body += v_body * dt;

    // cout << "trust: " << trust << endl;
    // cout << "ac x: " << a_filtered[0] << " y: " << a_filtered[1] << " z: " << a_filtered[2] << endl;
    // cout << "vel x: " << v_body[0] << " y: " << v_body[1] << " z: " << v_body[2] << endl;
    //cout << "x: " << p_body[0] << " y: " << p_body[1] << " z: " << p_body[2] << endl;
    // cout << (ros::Time::now() - time_start).toSec() << endl;
    z += vz_filtered2(2) * dt;
    // z += vz_filtered(2) * dt;
    //std::cout << "z: " << z << std::endl;
  }
}

template <typename T>
Vec3<T> PositionEstimator<T>::_filter(Vec3<T> acc, bool sw)
{
  Vec3<T> result(0, 0, 0);
  Vec3<T> sum(0, 0, 0);

  if (sw)
  {
    acc_buffer.push_back(acc);

    for (size_t i = 0; i < MOVING_AVERAGE; i++)
    {
      sum += acc_buffer.at(i);
    }
  }
  else
  {
    vel_buffer.push_back(acc);

    for (size_t i = 0; i < MOVING_AVERAGE; i++)
    {
      sum += vel_buffer.at(i);
    }
  }

  result = sum / (float)MOVING_AVERAGE;

  return result;
}

template class PositionEstimator<float>;