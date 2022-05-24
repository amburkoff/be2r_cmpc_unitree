#include "Metrics.h"

template <typename T>
void Metric<T>::setRobotData(ControlFSMData<T>& data)
{
_data = &data;
}

template <typename T>
SystemEnergy<T>::SystemEnergy()
{
_test << 1,2,3,4;

};
template <typename T>
Vec4<T> SystemEnergy<T>::getFinalLegCost()
{
_vBody = this._data._stateEstimator->getResult().vBody;
_position = this._data._stateEstimator.getResult().position;
_state_coord = this._data._legController->datas.p;
return _test;
};
template <typename T>
void SystemEnergy<T>::debugPrint() {};
// void ContactEnergy()
// {

// }
// void ~ContactEnergy()
// {
  
// }

// void ~SystemEnergy()
// {
  
// }
// void CostOfTrasport()
// {

// }
// void ~CostOfTrasport()
// {
  
// }
// void ContactEnergy::debugPrint() {}

