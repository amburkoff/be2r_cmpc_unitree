#ifndef ROS_READ_PARAM_H
#define ROS_READ_PARAM_H
#include <ros/ros.h>
namespace ros
{
//! @brief Шаблоннная функция для чтения параметров
template<typename T>
void readParam(const std::string param_name, T& param_value, const T default_value)
{
  if (!ros::param::get(param_name, param_value))
  {
    ROS_WARN_STREAM("Parameter \"" << param_name << "\" didn' find in Parameter Server."
                                   << "\nSetting default value: " << default_value);
    param_value = default_value;
  }
}
}

template<typename T>
bool readRosParam(std::string param_name, T& param_var)
{
  if (!ros::param::get(param_name, param_var))
  {
    ROS_WARN_STREAM("Can't read param " << param_name);
    return false;
  }
  // std::cout << "[ROS PARAM] " << param_name << ": " << param_var << std::endl;
  return true;
}

struct StaticParams
{
  void read()
  {
    readRosParam("/static_params/controller_dt", controller_dt);
    readRosParam("/static_params/foot_height_sensor_noise", foot_height_sensor_noise);
    readRosParam("/static_params/foot_process_noise_position", foot_process_noise_position);
    readRosParam("/static_params/foot_sensor_noise_position", foot_sensor_noise_position);
    readRosParam("/static_params/foot_sensor_noise_velocity", foot_sensor_noise_velocity);
    readRosParam("/static_params/imu_process_noise_position", imu_process_noise_position);
    readRosParam("/static_params/imu_process_noise_velocity", imu_process_noise_velocity);
  }
  double controller_dt;
  double foot_height_sensor_noise;
  double foot_process_noise_position;
  double foot_sensor_noise_position;
  double foot_sensor_noise_velocity;
  double imu_process_noise_position;
  double imu_process_noise_velocity;
};

#endif // ROS_READ_PARAM_H
