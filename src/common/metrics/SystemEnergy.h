#pragma once

#include <Metric.h>

using Eigen::Array4f;
using Eigen::Array4i;

// template <typename T>
class SystemEnergy : public Metric 
{
public:
  SystemEnergy();
  ~SystemEnergy() = default;

  Vec4<float> getFinalBodyCost();
  Vec4<float> getFinalCost();
  Vec4<float> getFinalLegCost();

  // void setRobotData(ControlFSMData<T>& data);
  // void debugPrint();
//   void earlyContactHandle(Vec4<uint8_t> , int , int ) {}
//   void restoreDefaults(){}
  Vec3<float> g;

private:
  Vec4<float> _test;
  Vec3<float> _vBody;
  Vec3<float> _position;
  Vec3<float> _state_coord;
  LegControllerData<float> datas[4];
  float _KinLinEnergy;
  float _KinRotEnergy;
  Vec4<float> _KinEnergyLeg;
  float _PotEnergy;
  Vec4<float> _PotEnergyLeg;
};
//   float _E = 0;
//   Vec3<float> _pBody_des;
//   Vec3<float> _vBody_des;
//   Vec3<float> _aBody_des;


//   Vec3<float> _pBody_RPY_des;
//   Vec3<float> _vBody_Ori_des;

//   Vec3<float> _pFoot_des[4];
//   Vec3<float> _vFoot_des[4];
//   Vec3<float> _aFoot_des[4];

//   Vec3<float> _Fr_des[4];

//   Vec4<float> _contact_state;


//   float _yaw_turn_rate;
//   float _yaw_des;

//   float _roll_des;
//   float _pitch_des;

//   float _x_vel_des = 0.;
//   float _y_vel_des = 0.;
//   float _z_vel_des = 0.;


// class ContactEnergy : public Metrics
// {
// public:
//   ContactEnergy(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
//   ~ContactEnergy()= default;
//   Vec4<float> getContactState();
//   Vec4<float> getSwingState();
//   Vec4<float> getFinalLegCost();

//   void debugPrint() {}
//   void earlyContactHandle(Vec4<uint8_t> , int , int ) {}
//   void restoreDefaults(){}

// private:

//   Vec3<float> _pBody_des;
//   Vec3<float> _vBody_des;
//   Vec3<float> _aBody_des;

//   Vec3<float> _pBody_RPY_des;
//   Vec3<float> _vBody_Ori_des;

//   Vec3<float> _pFoot_des[4];
//   Vec3<float> _vFoot_des[4];
//   Vec3<float> _aFoot_des[4];

//   Vec3<float> _Fr_des[4];

//   Vec4<float> _contact_state;

//   float _yaw_turn_rate;
//   float _yaw_des;

//   float _roll_des;
//   float _pitch_des;

//   float _x_vel_des = 0.;
//   float _y_vel_des = 0.;
//   float _z_vel_des = 0.;
// };

// class CostOfTrasport : public Metrics
// {
// public:
//   CostOfTrasport(int nSegment, Vec4<int> periods, float duty_cycle, const std::string& name);
//   ~CostOfTrasport()= default;

//   Vec4<float> getContactState();
//   Vec4<float> getSwingState();
//   Vec4<float> getFinalLegCost();
//   template <typename T>
//   void getRobotData(ControlFSMData<T>& data);
//   void debugPrint() {}
//   void earlyContactHandle(Vec4<uint8_t> , int , int ) {}
//   void restoreDefaults(){}

// private:
//   float _passed_lenght;
//   Vec3<float> _pBody_des;
//   Vec3<float> _vBody_des;
//   Vec3<float> _aBody_des;

//   Vec3<float> _pBody_RPY_des;
//   Vec3<float> _vBody_Ori_des;

//   Vec3<float> _pFoot_des[4];
//   Vec3<float> _vFoot_des[4];
//   Vec3<float> _aFoot_des[4];

//   Vec3<float> _Fr_des[4];

//   Vec4<float> _contact_state;

//   float _yaw_turn_rate;
//   float _yaw_des;

//   float _roll_des;
//   float _pitch_des;

//   float _x_vel_des = 0.;
//   float _y_vel_des = 0.;
//   float _z_vel_des = 0.;
// };
