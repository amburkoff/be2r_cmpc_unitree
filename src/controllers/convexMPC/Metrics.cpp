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
_KinEnergy << 0;

};
template <typename T>
Vec4<T> SystemEnergy<T>::getFinalLegCost()
{
_vBody = this._data._stateEstimator->getResult().vBody;
_KinEnergy = _vBody.transpose()*this._data._quadruped->_bodyMass*_vBody*T(0.5);
_position = this._data._stateEstimator.getResult().position;
_state_coord = this._data._legController->datas.p;
return _test;
std::cout << "Kinetic Energy"<< _KinEnergy<< "\n";
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

