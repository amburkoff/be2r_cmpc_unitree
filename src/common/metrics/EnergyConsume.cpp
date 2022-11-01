#include "EnergyConsume.h"

// This class calculates estimated cosumed power and energy by motors
EnergyConsume::EnergyConsume()
{

_LegPower.setZero();
_LegEnergy.setZero();
_dt = 0;

};


Vec4<float> EnergyConsume::getFinalCost()
{
  // Calcutes total cosumed energy
  return Vec4<float>(getFinalLegCost().sum(),0,0,0);
}

Vec4<float> EnergyConsume::getFinalBodyCost()
{
  // Calculates leg's consumed power
for (int i = 0; i < 4; i++)
  {
  Vec3<float> _tau_leg = this->_data->_legController->datas[i].tauEstimate;
  Vec3<float> _dq_leg = this->_data->_legController->datas[i].qd.transpose();
  _LegPower[i] = _dq_leg.dot(_tau_leg);

  }
return _LegPower;
}

Vec4<float> EnergyConsume::getFinalLegCost()
{
    // Calculates leg's consumed energy
_LegEnergy = this->_data->debug->metric_data.final_leg_cost;
_dt = this->_data->controlParameters->controller_dt;
_LegEnergy = _LegEnergy + getFinalBodyCost()*_dt;
return _LegEnergy;
}
// // template <typename T>
