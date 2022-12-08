#include "EnergyConsume.h"
// #include "SystemEnergy.h"

// This class calculates estimated cosumed power and energy by motors
EnergyConsume::EnergyConsume()
{

_LegPower.setZero();
_LegEnergy.setZero();
_dt = 0;

};

/*!
* Calculates Total Cosumed Energy
*/
Vec4<float> EnergyConsume::getFinalCost()
{
  return Vec4<float>(getFinalLegCost().sum(),0,0,0);
}
/*!
* Calculates leg's Сonsumed Power
*/
Vec4<float> EnergyConsume::getFinalBodyCost()
{
for (int i = 0; i < 4; i++)
  {
  Vec3<float> _tau_leg = this->_data->_legController->datas[i].tauEstimate;
  Vec3<float> _dq_leg = this->_data->_legController->datas[i].qd.transpose();
  _LegPower[i] = _dq_leg.dot(_tau_leg);

  }
return _LegPower;
}

/*!
* Calculates leg's Сonsumed Energy
*/
Vec4<float> EnergyConsume::getFinalLegCost()
{

_LegEnergy = this->_data->debug->metric_data.final_leg_cost;
_dt = this->_data->staticParams->controller_dt;
_LegEnergy = _LegEnergy + getFinalBodyCost()*_dt;
return _LegEnergy;
}
// // template <typename T>
