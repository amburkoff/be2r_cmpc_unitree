#pragma once

#include "Metric.h"

using Eigen::Array4f;
using Eigen::Array4i;

// template <typename T>
class CostFunc_MPC : public Metric 
{
public:
  CostFunc_MPC();
  ~CostFunc_MPC() = default;

  Vec4<float> getFinalBodyCost();
  Vec4<float> getFinalCost();
  Vec4<float> getFinalLegCost();

  // void setRobotData(ControlFSMData<T>& data);
  // void debugPrint();
//   void earlyContactHandle(Vec4<uint8_t> , int , int ) {}
//   void restoreDefaults(){}
  Vec3<float> g;

private:
  Eigen::Matrix<float, 12, 12> _Q;
  Eigen::Matrix<float, 12, 12> _R;
  Vec12<float> _state;
  Vec12<float> _forces;
  LegControllerData<float> datas[4];
  float _Cost2go;
  Vec12<float> _Gradient;

};
