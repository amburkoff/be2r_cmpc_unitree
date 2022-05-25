#ifndef PROJECT_METRICS_H
#define PROJECT_METRICS_H

#include <queue>
#include <string>
#include <iostream>
#include "cppTypes.h"
#include <cmath>
#include <ControlFSMData.h>
// #include "LegController.h"

template <typename T>
class Metric
{
public:
  virtual ~Metric() = default;
  virtual Vec4<T> getContactState() = 0;
  virtual Vec4<T> getSwingState() = 0;
  virtual Vec4<T> getFinalLegCost() = 0;
  void setRobotData(ControlFSMData<T>& data);
  virtual void debugPrint();
//   virtual void earlyContactHandle(Vec4<uint8_t> , int , int ) {}
//   virtual void restoreDefaults(){}
  
  ControlFSMData<T>* _data;
protected:
  std::string _name;
  
};

using Eigen::Array4f;
using Eigen::Array4i;

template <typename T>
class SystemEnergy : public Metric<T> 
{
public:
  SystemEnergy();
  ~SystemEnergy()= default;

  Vec4<T> getContactState();
  Vec4<T> getSwingState();
  Vec4<T> getFinalLegCost();

  void setRobotData(ControlFSMData<T>& data);
  void debugPrint();
//   void earlyContactHandle(Vec4<uint8_t> , int , int ) {}
//   void restoreDefaults(){}

private:
  Vec4<T> _test;
  Vec3<T> _vBody;
  Vec3<T> _position;
  Vec3<T> _state_coord;
  LegControllerData<T> datas[4];
  T _KinEnergy;
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


#endif //PROJECT_METRICS_H