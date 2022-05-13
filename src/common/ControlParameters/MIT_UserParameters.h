#ifndef PROJECT_MITUSERPARAMETERS_H
#define PROJECT_MITUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class MIT_UserParameters : public ControlParameters
{
public:
  MIT_UserParameters()
    : ControlParameters("user-parameters")
    , INIT_PARAMETER(use_wbc)
    , INIT_PARAMETER(Kp_joint)
    , INIT_PARAMETER(Kd_joint)
    , INIT_PARAMETER(Kp_ori)
    , INIT_PARAMETER(Kd_ori)
    , INIT_PARAMETER(Kp_body)
    , INIT_PARAMETER(Kd_body)
    , INIT_PARAMETER(Kp_foot)
    , INIT_PARAMETER(Kd_foot)
    , INIT_PARAMETER(cmpc_gait)
    , INIT_PARAMETER(cmpc_x_drag)
    , INIT_PARAMETER(cmpc_use_sparse)
    , INIT_PARAMETER(cmpc_bonus_swing)
    , INIT_PARAMETER(jcqp_alpha)
    , INIT_PARAMETER(jcqp_max_iter)
    , INIT_PARAMETER(jcqp_rho)
    , INIT_PARAMETER(jcqp_sigma)
    , INIT_PARAMETER(jcqp_terminate)
    , INIT_PARAMETER(use_jcqp)
    , INIT_PARAMETER(Swing_Kp_cartesian)
    , INIT_PARAMETER(Swing_Kd_cartesian)
    , INIT_PARAMETER(Swing_traj_height)
    , INIT_PARAMETER(stance_legs)
    , INIT_PARAMETER(gait_type)
    , INIT_PARAMETER(gait_period_time)
    , INIT_PARAMETER(gait_switching_phase)
    , INIT_PARAMETER(gait_override)
  {
  }

  DECLARE_PARAMETER(double, use_wbc);

  DECLARE_PARAMETER(Vec3<double>, Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint);

  DECLARE_PARAMETER(Vec3<double>, Kp_ori);
  DECLARE_PARAMETER(Vec3<double>, Kd_ori);

  DECLARE_PARAMETER(Vec3<double>, Kp_body);
  DECLARE_PARAMETER(Vec3<double>, Kd_body);

  DECLARE_PARAMETER(Vec3<double>, Kp_foot);
  DECLARE_PARAMETER(Vec3<double>, Kd_foot);

  DECLARE_PARAMETER(double, cmpc_gait);
  DECLARE_PARAMETER(double, cmpc_x_drag);
  DECLARE_PARAMETER(double, cmpc_use_sparse);
  DECLARE_PARAMETER(double, cmpc_bonus_swing);

  DECLARE_PARAMETER(double, jcqp_alpha);
  DECLARE_PARAMETER(double, jcqp_max_iter);
  DECLARE_PARAMETER(double, jcqp_rho);
  DECLARE_PARAMETER(double, jcqp_sigma);
  DECLARE_PARAMETER(double, jcqp_terminate);
  DECLARE_PARAMETER(double, use_jcqp);

  // Swing leg parameters
  DECLARE_PARAMETER(Vec3<double>, Swing_Kp_cartesian);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kd_cartesian);
  DECLARE_PARAMETER(double, Swing_traj_height);

  DECLARE_PARAMETER(double, stance_legs);

  // Gait Scheduler
  DECLARE_PARAMETER(double, gait_type);
  DECLARE_PARAMETER(double, gait_period_time);
  DECLARE_PARAMETER(double, gait_switching_phase);
  DECLARE_PARAMETER(double, gait_override);
};

#endif // PROJECT_MITUSERPARAMETERS_H
