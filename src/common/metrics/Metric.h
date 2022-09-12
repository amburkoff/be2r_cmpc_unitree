#pragma once

#include <queue>
#include <string>
#include <iostream>
#include "cppTypes.h"
#include <cmath>
#include <ControlFSMData.h>
#include "Dynamics/SpatialInertia.h"
// #include "LegController.h"

class Metric
{
public:
  virtual ~Metric() = default;
  virtual Vec4<float> getFinalBodyCost() = 0; 
  virtual Vec4<float> getFinalCost() = 0;
  virtual Vec4<float> getFinalLegCost() = 0; //0 - means 'have to be specified', {} -- means 'if not specified will be {}'
  void setRobotData(ControlFSMData<float>& data);
  void computeCenterLegVelAndPos(Quadruped<float>& quad, Vec3<float>& q, Vec3<float>& dq, Mat3<float>* J, Mat3<float>* leg_p, Mat3<float>* Leg_v, Mat3<float>* Leg_w, int leg);
  void debugPrint();
//   virtual void earlyContactHandle(Vec4<uint8_t> , int , int ) {}
//   virtual void restoreDefaults(){}
  
  ControlFSMData<float>* _data;
protected:
  std::string _name;
  
};

