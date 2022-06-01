#ifndef CHEETAH_SOFTWARE_VISION_MPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_VISION_MPCLOCOMOTION_H

#include "cppTypes.h"
#include <ControlFSMData.h>
#include <Controllers/FootSwingTrajectory.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <Utilities/SpiralIterator.hpp>

using Eigen::Array4f;
using Eigen::Array4i;

// Step height maximum [m]
#define MAX_STEP_HEIGHT 0.20

class VisionGait
{
public:
  VisionGait(int nMPC_segments, Vec4<int> offsets, Vec4<int> durations,
             const std::string& name = "");
  ~VisionGait();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* mpc_gait();
  void setIterations(int iterationsPerMPC, int currentIteration);
  int getCurrentGaitPhase() { return _iteration; }
  int _stance;
  int _swing;

private:
  int _nMPC_segments;
  int* _mpc_table;
  Array4i _offsets;        // offset in mpc segments
  Array4i _durations;      // duration of step in mpc segments
  Array4f _offsetsFloat;   // offsets in phase (0 to 1)
  Array4f _durationsFloat; // durations in phase (0 to 1)
  int _iteration;
  int _nIterations;
  float _phase;
};

class VisionMPCLocomotion
{
public:
  VisionMPCLocomotion(float _dt, int _iterations_between_mpc,
                      be2r_cmpc_unitree::ros_dynamic_paramsConfig* parameters);
  void initialize();

  void run(ControlFSMData<float>& data, const Vec3<float>& vel_cmd,
           const grid_map::GridMap& height_map, const grid_map::GridMap& height_map_raw);

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;

private:
  void _updateFoothold(Vec3<float>& foot, const Vec3<float>& body_pos,
                       const grid_map::GridMap& height_map,
                       const grid_map::GridMap& height_map_raw, int leg);
  void _IdxMapChecking(Vec3<float>& Pf, int x_idx, int y_idx, int& x_idx_selected,
                       int& y_idx_selected, const grid_map::GridMap& height_map, int leg);
  void _updateParams(ControlFSMData<float>& data);
  float _updateTrajHeight(size_t foot);

  be2r_cmpc_unitree::ros_dynamic_paramsConfig* _parameters = nullptr;

  Vec3<float> _fin_foot_loc[4];
  float grid_size = 0.02;

  Vec3<float> v_des_world;
  Vec3<float> rpy_des;
  Vec3<float> v_rpy_des;

  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data);
  void solveDenseMPC(int* mpcTable, ControlFSMData<float>& data);
  int iterationsBetweenMPC;
  float _body_height;
  int _gait_period;
  int horizonLength;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  VisionGait trotting, bounding, pronking, galloping, standing, trotRunning, walking;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  Vec3<float> pFoot[4];
  float trajAll[12 * 36];
  ControlFSMData<float>* _data;
};

#endif // CHEETAH_SOFTWARE_VISION_MPCLOCOMOTION_H
