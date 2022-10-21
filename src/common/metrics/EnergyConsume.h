#pragma once

#include "Metric.h"

using Eigen::Array4f;
using Eigen::Array4i;

// template <typename T>
class EnergyConsume: public Metric 
{
public:
  EnergyConsume();
  ~EnergyConsume() = default;

  Vec4<float> getFinalBodyCost();
  Vec4<float> getFinalCost();
  Vec4<float> getFinalLegCost();
 

private:
  Vec4<float> _LegPower;
  Vec4<float> _LegEnergy;
  float _dt;
};