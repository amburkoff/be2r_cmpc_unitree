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

#endif // ROS_READ_PARAM_H
