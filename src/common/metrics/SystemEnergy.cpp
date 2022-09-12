#include "SystemEnergy.h"


// template <typename float>
SystemEnergy::SystemEnergy()
{
  g = Vec3<float>(0, 0, -9.81);
  _KinLinEnergy = float(0);
  _KinRotEnergy = float(0);
  _PotEnergy = float(0);
  _KinEnergyLeg.setZero();
  _PotEnergyLeg.setZero();
};

Vec4<float> SystemEnergy::getFinalCost()
{
  
  return Vec4<float>(getFinalBodyCost().operator()(3) + getFinalLegCost().sum(),0,0,0);
}

Vec4<float> SystemEnergy::getFinalBodyCost()
{
  _vBody = this->_data->_stateEstimator->getResult().vBody;
  _KinLinEnergy = _vBody.dot(_vBody) * this->_data->_quadruped->_bodyMass * float(0.5);
  _position = this->_data->_stateEstimator->getResult().position;
  _PotEnergy = this->_data->_quadruped->_bodyMass * _position[2] * g[2];
  Vec4<float> Result;
  _KinRotEnergy = float(0.5)*this->_data->_stateEstimator->getResult().omegaBody.dot(this->_data->_quadruped->_bodyInertia.getInertiaTensor()*this->_data->_stateEstimator->getResult().omegaBody);
  Result.setZero();
  Result.operator()(0) = _KinLinEnergy;
  Result.operator()(1) = _KinRotEnergy;
  Result.operator()(2) = _PotEnergy;
  Result.operator()(3) = _KinLinEnergy + _KinRotEnergy + _PotEnergy;
return Result;
}
Vec4<float> SystemEnergy::getFinalLegCost()
{
  
  Mat3<float> Rbod = this->_data->_stateEstimator->getResult().rBody.transpose();

  Mat3<float> Jacobi;
  Mat3<float> globalCOMp_leg;
  Mat3<float> globalCOMv_leg;
  Mat3<float> globalCOMw_leg;
  Jacobi.setZero();
  globalCOMp_leg.setZero();
  globalCOMv_leg.setZero();
  globalCOMw_leg.setZero();
  _KinEnergyLeg.setZero();
  _PotEnergyLeg.setZero();
  for (int i = 0; i < 4; i++)
  {
    Quadruped<float> &quadruped = *this->_data->_quadruped;
    // compute velocities and positions of the leg COM in the local fixed hip frame 
    Metric::computeCenterLegVelAndPos(quadruped,this->_data->_legController->datas[i].q,this->_data->_legController->datas[i].qd,&(Jacobi),&(globalCOMp_leg),&(globalCOMv_leg),&(globalCOMw_leg),i);
    
    // Body COM velocity and position in the global frame
    _position = this->_data->_stateEstimator->getResult().position;
    _vBody = this->_data->_stateEstimator->getResult().vBody;

    Vec3<float> ph = quadruped.getHipLocation(i); // hip positions relative to CoM

    Vec3<float> p_rel = ph + this->_data->_legController->datas[i].p; // Local frame distance from COM to leg(i)
    
    // Local frame velocity of leg(i) relative to COM
    Vec3<float> dp_rel = this->_data->_legController->datas[i].v;

    // Distance to leg(i) from Aligned with World frame body frame
    Vec3<float> p_f = Rbod * p_rel;

    // World leg(i) velocity in Aligned with World frame body frame
    Vec3<float> dp_f = Rbod * (this->_data->_stateEstimator->getResult().omegaBody.cross(p_rel) + dp_rel);

    // std::cout<< globalCOMp_leg.block(0,0,1,1)<< endl;
    // Move from i-th leg hip local frame to the body frame
    globalCOMp_leg.block(0,0,3,1) += ph;//globalCOMp_leg.block(0,0,3,1) + 
    globalCOMp_leg.block(0,1,3,1) += ph;
    globalCOMp_leg.block(0,2,3,1) += ph;
    
    // added linear velocity component from body rotations
    globalCOMv_leg.block(0,0,3,1) += this->_data->_stateEstimator->getResult().omegaBody.cross(Vec3<float>(globalCOMp_leg.block(0,0,3,1)));
    globalCOMv_leg.block(0,1,3,1) += this->_data->_stateEstimator->getResult().omegaBody.cross(Vec3<float>(globalCOMp_leg.block(0,1,3,1)));
    globalCOMv_leg.block(0,2,3,1) += this->_data->_stateEstimator->getResult().omegaBody.cross(Vec3<float>(globalCOMp_leg.block(0,2,3,1)));
    
    // Added body angular velocity
    globalCOMw_leg.block(0,0,3,1) = globalCOMw_leg.block(0,0,3,1) + this->_data->_stateEstimator->getResult().omegaBody;
    globalCOMw_leg.block(0,1,3,1) = globalCOMw_leg.block(0,1,3,1) + this->_data->_stateEstimator->getResult().omegaBody;
    globalCOMw_leg.block(0,2,3,1) = globalCOMw_leg.block(0,2,3,1) + this->_data->_stateEstimator->getResult().omegaBody;
    
    // Rotated, World leg(i) COM positions and velocities are expressed 
    // // in Aligned with World frame body frame
    globalCOMv_leg.block(0,0,3,3) = Rbod * globalCOMv_leg;
    globalCOMp_leg.block(0,0,3,3) = Rbod * globalCOMv_leg;
    globalCOMw_leg.block(0,0,3,3) = Rbod * globalCOMv_leg;
    
    // Expressed in the global frame
    globalCOMv_leg.block(0,0,3,1) += _vBody;
    globalCOMv_leg.block(0,1,3,1) += _vBody;
    globalCOMv_leg.block(0,2,3,1) += _vBody;

    // // Global position of the COM for the leg Expressed in the world frame (initial frame)
    globalCOMp_leg.block(0,0,3,1) += _position;
    globalCOMp_leg.block(0,1,3,1) += _position;
    globalCOMp_leg.block(0,2,3,1) += _position;

    // Sum of the kinetic rotational and linear energy for all leg bodies
    _KinEnergyLeg.operator()(i) = Vec3<float>(globalCOMw_leg.block(0,0,3,1)).dot(quadruped._abadInertia.getInertiaTensor()*Vec3<float>(globalCOMw_leg.block(0,0,3,1))) + Vec3<float>(globalCOMv_leg.block(0,0,3,1)).dot(Vec3<float>(globalCOMv_leg.block(0,0,3,1)))*quadruped._abadInertia.getMass();
    _KinEnergyLeg.operator()(i) += Vec3<float>(globalCOMw_leg.block(0,1,3,1)).dot(quadruped._hipInertia.getInertiaTensor()*Vec3<float>(globalCOMw_leg.block(0,1,3,1))) + Vec3<float>(globalCOMv_leg.block(0,1,3,1)).dot(Vec3<float>(globalCOMv_leg.block(0,1,3,1)))*quadruped._hipInertia.getMass();
    _KinEnergyLeg.operator()(i) += Vec3<float>(globalCOMw_leg.block(0,2,3,1)).dot(quadruped._kneeInertia.getInertiaTensor()*Vec3<float>(globalCOMw_leg.block(0,2,3,1))) + Vec3<float>(globalCOMv_leg.block(0,2,3,1)).dot(Vec3<float>(globalCOMv_leg.block(0,2,3,1)))*quadruped._kneeInertia.getMass();

    // Sum of the potential rot and lin energy for all leg bodies
    _PotEnergyLeg.operator()(i) = globalCOMp_leg.operator()(2,0)*quadruped._abadInertia.getMass();
    _PotEnergyLeg.operator()(i) += globalCOMp_leg.operator()(2,1)*quadruped._hipInertia.getMass();
    _PotEnergyLeg.operator()(i) += globalCOMp_leg.operator()(2,2)*quadruped._kneeInertia.getMass();
  }
  _KinEnergyLeg = float(0.5) * _KinEnergyLeg;
  _PotEnergyLeg = g[2] * _PotEnergyLeg;
  // _state_coord = this->_data->._legController->datas.p;
  // std::cout << "Kinetic Energy" << _KinLinEnergy << "\n";
  //  a(_KinLinEnergy +_KinRotEnergy + _PotEnergy, 0, 0, 0);
  return Vec4<float>(_KinEnergyLeg + _PotEnergyLeg);
};
// template <typename T>



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
