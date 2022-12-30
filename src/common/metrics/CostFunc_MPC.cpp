#include "CostFunc_MPC.h"



CostFunc_MPC::CostFunc_MPC()
{
  _state.setZero();
  _forces.setZero();
  _Cost2go = 0;
  _Gradient.setZero();
  _Q.setZero();
  _R.setZero();
  float alpha = 4e-5; 
  float Qd[12] = { 2.5, 2.5, 10, 50, 50, 100, 0, 0, 0.5, 0.2, 0.2, 0.1 };
  for (int i=0; i < 12; i++)
  {
    // _Q.block(i,i,1,1) = Qd(i);
    // _R.block(i,i,1,1) = alpha;
  }

};

Vec4<float> CostFunc_MPC::getFinalCost()
{
  
  return Vec4<float>(0,0,0,0);
}

Vec4<float> CostFunc_MPC::getFinalBodyCost()
{
return Vec4<float>(0,0,0,0);
}

Vec4<float> CostFunc_MPC::getFinalLegCost()
{
  
  Vec4<uint8_t> contactState = _data->_stateEstimator->getContactSensorData();
  Vec4<float> dE(0,0,0,0);
  return dE;
};