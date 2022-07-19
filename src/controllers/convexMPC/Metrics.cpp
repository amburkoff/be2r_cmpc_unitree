#include "Metrics.h"

// template <typename T>
void Metric::setRobotData(ControlFSMData<float>& data)
{
_data = &data;
}
void Metric::computeCenterLegVelAndPos(Quadruped<float>& quad, Vec3<float>& q, Vec3<float>& dq, Mat3<float>* J, Vec3<float>* p, int leg)
{
  float l1 = quad._abadLinkLength;
  float l2 = quad._hipLinkLength;
  float l3 = quad._kneeLinkLength;
  float l4 = quad._kneeLinkY_offset;
  SpatialInertia<float> _abad = quad._abadInertia;
  SpatialInertia<float> _hip = quad._hipInertia;
  SpatialInertia<float> _knee = quad._kneeInertia;
  float sideSign = quad.getSideSign(leg);

  float s1 = std::sin(q(0));
  float s2 = std::sin(q(1));
  float s3 = std::sin(q(2));

  float c1 = std::cos(q(0));
  float c2 = std::cos(q(1));
  float c3 = std::cos(q(2));

  float c23 = c2 * c3 - s2 * s3;
  float s23 = s2 * c3 + c2 * s3;

  if (J)
  {
    J->operator()(0, 0) = 0;
    J->operator()(0, 1) = l3 * c23 + l2 * c2;
    J->operator()(0, 2) = l3 * c23;
    J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1;
    J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) = l3 * c1 * s23;
  }

  if (p)
  {
    p->operator()(0) = l3 * s23 + l2 * s2;
    p->operator()(1) = (l1 + l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = (l1 + l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}
// template <typename float>
SystemEnergy::SystemEnergy()
{
_test << float(1),float(2),float(3),float(4);
_KinEnergy = float(0);
_PotEnergy = float(0);
_KinEnergyLeg.setZero();
_PotEnergyLeg.setZero();


};

// template <typename float>
Vec4<float> SystemEnergy::getFinalLegCost()
{
_vBody = this->_data->_stateEstimator->getResult().vBody;
_KinEnergy = _vBody.dot(_vBody) * this->_data->_quadruped->_bodyMass * float(0.5);
_position = this->_data->_stateEstimator->getResult().position;
Vec3<float> g(0,0,9.81);
_PotEnergy = this->_data->_quadruped->_bodyMass*_position[2]*g[2];
Mat3<float> Rbod = this->_data->_stateEstimator->getResult().rBody.transpose();

for (int i = 0; i < 4; i++)
    {
Quadruped<float>* quadruped = this->_data->_quadruped;
Vec3<float> ph = quadruped->getHipLocation(i); // hip positions relative to CoM
// hw_i->leg_controller->leg_datas[i].p;
Vec3<float> p_rel = ph + this->_data->_legController->datas[i].p; // Local frame distance from COM to leg(i)
// hw_i->leg_controller->leg_datas[i].v;

// Local frame velocity of leg(i) relative to COM
Vec3<float> dp_rel = this->_data->_legController->datas[i].v;

//Distance to leg(i) from Aligned with World frame body frame
Vec3<float> p_f = Rbod * p_rel; 

// World leg(i) velocity in Aligned with World frame body frame
Vec3<float> dp_f = Rbod * (this->_data->_stateEstimator->getResult().omegaBody.cross(p_rel) + dp_rel);
}
// _state_coord = this->_data->._legController->datas.p;
std::cout << "Kinetic Energy"<< _KinEnergy<< "\n";
Vec4<float> a(_KinEnergy+_PotEnergy,0,0,0);
return a;

};
// template <typename T>
void SystemEnergy::debugPrint() {};

// template <typename T>
Vec4<float> SystemEnergy::getContactState() {
    Vec4<float> a(0,0,0,0);
    return a;
};

// template <typename T>
Vec4<float> SystemEnergy::getSwingState() {
    Vec4<float> a(0,0,0,0);
    return a;
};
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

