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
    readRosParam("/static_params/horizon", horizon);
    readRosParam("/static_params/foot_height_sensor_noise", foot_height_sensor_noise);
    readRosParam("/static_params/foot_process_noise_position", foot_process_noise_position);
    readRosParam("/static_params/foot_sensor_noise_position", foot_sensor_noise_position);
    readRosParam("/static_params/foot_sensor_noise_velocity", foot_sensor_noise_velocity);
    readRosParam("/static_params/imu_process_noise_position", imu_process_noise_position);
    readRosParam("/static_params/imu_process_noise_velocity", imu_process_noise_velocity);
    readRosParam("/static_params/cheater_mode", cheater_mode);
    readRosParam("/static_params/Q_roll", Q_roll);
    readRosParam("/static_params/Q_pitch", Q_pitch);
    readRosParam("/static_params/Q_yaw", Q_yaw);
    readRosParam("/static_params/Q_x", Q_x);
    readRosParam("/static_params/Q_y", Q_y);
    readRosParam("/static_params/Q_z", Q_z);
    readRosParam("/static_params/Q_w_roll", Q_w_roll);
    readRosParam("/static_params/Q_w_pitch", Q_w_pitch);
    readRosParam("/static_params/Q_w_yaw", Q_w_yaw);
    readRosParam("/static_params/Q_vx", Q_vx);
    readRosParam("/static_params/Q_vy", Q_vy);
    readRosParam("/static_params/Q_vz", Q_vz);
    readRosParam("/static_params/alpha", alpha);

    readRosParam("/static_params/mpc_alpha", mpc_alpha);
    std::cout << "mpc alfa: " << mpc_alpha << std::endl;

    readRosParam("/static_params/max_vel_x", max_vel_x);
    readRosParam("/static_params/max_vel_y", max_vel_y);
    readRosParam("/static_params/max_turn_rate", max_turn_rate);
    readRosParam("/static_params/pitch_cor_max", pitch_cor_max);
    readRosParam("/static_params/pitch_cor", pitch_cor);
    readRosParam("/static_params/pitch_cor_min", pitch_cor_min);
    readRosParam("/static_params/iterations_between_mpc", iterations_between_mpc);
    readRosParam("/static_params/gamepad_filter", gamepad_filter);

    // readRosParam("/bigpid_params/Px", Px);
    // readRosParam("/bigpid_params/Py", Py);
    // readRosParam("/bigpid_params/Pz", Pz);
    // readRosParam("/bigpid_params/Dx", Dx);
    // readRosParam("/bigpid_params/Dy", Dy);
    // readRosParam("/bigpid_params/Dz", Dz);
    // readRosParam("/bigpid_params/P_roll", P_roll);
    // readRosParam("/bigpid_params/P_pitch", P_pitch);
    // readRosParam("/bigpid_params/P_yaw", P_yaw);
    // readRosParam("/bigpid_params/D_roll", D_roll);
    // readRosParam("/bigpid_params/D_pitch", D_pitch);
    // readRosParam("/bigpid_params/D_yaw", D_yaw);
  }

  double controller_dt;
  bool cheater_mode;
  double foot_height_sensor_noise;
  double foot_process_noise_position;
  double foot_sensor_noise_position;
  double foot_sensor_noise_velocity;
  double imu_process_noise_position;
  double imu_process_noise_velocity;
  int horizon;

  float alpha = 4e-5;
  float Q_roll = 10;
  float Q_pitch = 10;
  float Q_yaw = 15;
  float Q_x = 3;
  float Q_y = 3;
  float Q_z = 30;
  float Q_w_roll = 0.5;
  float Q_w_pitch = 0.5;
  float Q_w_yaw = 3;
  float Q_vx = 0.4;
  float Q_vy = 0.4;
  float Q_vz = 0.2;

  float mpc_alpha = 4e-5;

  float max_vel_x = 0.5;
  float max_vel_y = 0.4;
  float max_turn_rate = 2.5;
  float pitch_cor_max = -0.3;
  float pitch_cor = 0;
  float pitch_cor_min = -0.3;
  int iterations_between_mpc = 13;
  float gamepad_filter = 0.001;

  float Px = 1;
  float Py = 1;
  float Pz = 1;
  float Dx = 1;
  float Dy = 1;
  float Dz = 1;

  float P_roll = 1;
  float P_pitch = 100;
  float P_yaw = 1;
  float D_roll = 1;
  float D_pitch = 1;
  float D_yaw = 1;
};

#endif // ROS_READ_PARAM_H
