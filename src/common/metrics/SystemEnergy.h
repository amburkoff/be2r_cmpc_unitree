#pragma once

#include "Metric.h"

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