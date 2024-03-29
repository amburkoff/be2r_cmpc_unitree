#pragma once

class qp_controller_data_t
{
public:
  double exit_flag;

  double nWSR;

  double cpu_time_microseconds;

  double xOpt[12];

  double p_des[3];

  double p_act[3];

  double v_des[3];

  double v_act[3];

  double O_err[3];

  double omegab_des[3];

  double omegab_act[3];

  double lbA[20];

  double ubA[20];

  double C_times_f[20];

  double b_control[6];

  double b_control_Opt[6];

  double active;

  double pfeet_des[12];

  double pfeet_act[12];
};
