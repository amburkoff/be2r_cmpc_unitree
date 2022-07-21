#include "Metrics.h"

// template <typename T>
void Metric::setRobotData(ControlFSMData<float> &data)
{
  _data = &data;
}
void Metric::debugPrint()
{
  // _data->debug->metric_data.final_body_cost = getFinalBodyCost();
  // _data->debug->metric_data.final_leg_cost = getFinalLegCost();
  // _data->debug->metric_data.final_cost = getFinalCost();
};

void Metric::computeCenterLegVelAndPos(Quadruped<float> &quad, Vec3<float> &q, Vec3<float> &dq, Mat3<float> *J, Mat3<float> *leg_p, Mat3<float> *leg_v, Mat3<float> *leg_w,const int leg)
{
  // Computes in the hip coordinate frame linear/angular velocity and position for legs' COMs
  // As an extra caculates Jacobi matrix for the end-effector

  // leg_p = |_abadCOM_position,_hipCOM_position,_kneeCOM_position|
  // leg_v,leg_w the same as leg_p

  float l1 = quad._abadLinkLength;
  float l2 = quad._hipLinkLength;
  float l3 = quad._kneeLinkLength;
  float l4 = quad._kneeLinkY_offset;
  Vec3<float> _abadCOM = quad._abadInertia.getCOM();
  Vec3<float> _hipCOM = quad._hipInertia.getCOM();
  Vec3<float> _kneeCOM = quad._kneeInertia.getCOM();

  assert((leg <= 3) && (leg >= 0)); // four legs check
  // get direction of abad and knee joints offset for the left and right side of the robot
  float sideSign = quad.getSideSign(leg);

  Mat3<float> R1 = coordinateRotation(CoordinateAxis::X,q(0));
  Mat3<float> R2 = coordinateRotation(CoordinateAxis::Z,q(1));
  Mat3<float> R3 = coordinateRotation(CoordinateAxis::Z,q(2));
  
  Mat3<float> w1 = vectorToSkewMat(Vec3<float>(dq(0),0,0));
  Mat3<float> w2 = vectorToSkewMat(Vec3<float>(0,dq(1),0));
  Mat3<float> w3 = vectorToSkewMat(Vec3<float>(0,dq(2),0));

  if ((leg_p) && (leg_v) && (leg_w))
  {
    leg_p->block(0,0,3,1) = R1*sideSign*_abadCOM;
    leg_p->block(0,1,3,1) = R1*(Vec3<float>(0,-sideSign*l1,0) + R2*_hipCOM);
    leg_p->block(0,2,3,1) = R1*(Vec3<float>(0,-sideSign*(l1+l4),0) + R2*(Vec3<float>(0,0,-l2) + R3*_kneeCOM));

    leg_v->block(0,0,3,1) = w1*R1*sideSign*_abadCOM;
    leg_v->block(0,1,3,1) = w1*R1*(Vec3<float>(0,-sideSign*l1,0) + R2*_hipCOM) + R1*w2*R2*_hipCOM;
    leg_v->block(0,2,3,1) = w1*R1*(Vec3<float>(0,-sideSign*(l1+l4),0) + R2*(Vec3<float>(0,0,-l2) + R3*_kneeCOM)) + R1*w2*R2*(Vec3<float>(0,0,-l2) + R3*_kneeCOM)+ R1*R2*w3*R3*_kneeCOM;

    leg_w->block(0,0,3,1) = matToSkewVec(w1);
    leg_w->block(0,1,3,1) = matToSkewVec(w1) + R1*matToSkewVec(w2);
    leg_w->block(0,2,3,1) = matToSkewVec(w1) + R1*(matToSkewVec(w2)+matToSkewVec(w3));
  }
  
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
}
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
  
  return Vec4<float>(getFinalBodyCost().sum() + getFinalLegCost().sum(),0,0,0);
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
return Result;
}
Vec4<float> SystemEnergy::getFinalLegCost()
{
  
  Mat3<float> Rbod = this->_data->_stateEstimator->getResult().rBody.transpose();

  Mat3<float>* Jacobi;
  Mat3<float>* globalCOMp_leg;
  Mat3<float>* globalCOMv_leg;
  Mat3<float>* globalCOMw_leg;
  Jacobi->setZero();
  globalCOMp_leg->setZero();
  globalCOMv_leg->setZero();
  globalCOMw_leg->setZero();
  _KinEnergyLeg.setZero();
  _PotEnergyLeg.setZero();
  for (int i = 0; i < 4; i++)
  {
    Quadruped<float> &quadruped = *this->_data->_quadruped;
    // compute velocities and positions of the leg COM in the local fixed hip frame 
    Metric::computeCenterLegVelAndPos(quadruped,this->_data->_legController->datas[i].q,this->_data->_legController->datas[i].qd,Jacobi,globalCOMp_leg,globalCOMv_leg,globalCOMw_leg,i);
    
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


    // Move from i-th leg hip local frame to the body frame
    globalCOMp_leg->block(0,0,3,1) += ph;
    globalCOMp_leg->block(0,1,3,1) += ph;
    globalCOMp_leg->block(0,2,3,1) += ph;
    
    // added linear velocity component from body rotations
    globalCOMv_leg->block(0,0,3,1) += this->_data->_stateEstimator->getResult().omegaBody.cross(Vec3<float>(globalCOMp_leg->block(0,0,3,1)));
    globalCOMv_leg->block(0,1,3,1) += this->_data->_stateEstimator->getResult().omegaBody.cross(Vec3<float>(globalCOMp_leg->block(0,1,3,1)));
    globalCOMv_leg->block(0,2,3,1) += this->_data->_stateEstimator->getResult().omegaBody.cross(Vec3<float>(globalCOMp_leg->block(0,2,3,1)));
    
    // Added body angular velocity
    globalCOMw_leg->block(0,0,3,1) = globalCOMw_leg->block(0,0,3,1) + this->_data->_stateEstimator->getResult().omegaBody;
    globalCOMw_leg->block(0,1,3,1) = globalCOMw_leg->block(0,1,3,1) + this->_data->_stateEstimator->getResult().omegaBody;
    globalCOMw_leg->block(0,2,3,1) = globalCOMw_leg->block(0,2,3,1) + this->_data->_stateEstimator->getResult().omegaBody;
    
    // Rotated, World leg(i) COM positions and velocities are expressed 
    // in Aligned with World frame body frame
    globalCOMv_leg->block(0,0,3,3) = Rbod * globalCOMv_leg->block(0,0,3,3);
    globalCOMp_leg->block(0,0,3,3) = Rbod * globalCOMp_leg->block(0,0,3,3);
    globalCOMw_leg->block(0,0,3,3) = Rbod * globalCOMw_leg->block(0,0,3,3);
    
    // Expressed in the global frame
    globalCOMv_leg->block(0,0,3,1) += _vBody;
    globalCOMv_leg->block(0,1,3,1) += _vBody;
    globalCOMv_leg->block(0,2,3,1) += _vBody;

    // // Global position of the COM for the leg Expressed in the world frame (initial frame)
    globalCOMp_leg->block(0,0,3,1) += _position;
    globalCOMp_leg->block(0,1,3,1) += _position;
    globalCOMp_leg->block(0,2,3,1) += _position;

    // Sum of the kinetic rotational and linear energy for all leg bodies
    _KinEnergyLeg.operator()(i) = Vec3<float>(globalCOMw_leg->block(0,0,3,1)).dot(quadruped._abadInertia.getInertiaTensor()*Vec3<float>(globalCOMw_leg->block(0,0,3,1))) + Vec3<float>(globalCOMv_leg->block(0,0,3,1)).dot(Vec3<float>(globalCOMv_leg->block(0,0,3,1)))*quadruped._abadInertia.getMass();
    _KinEnergyLeg.operator()(i) += Vec3<float>(globalCOMw_leg->block(0,1,3,1)).dot(quadruped._hipInertia.getInertiaTensor()*Vec3<float>(globalCOMw_leg->block(0,1,3,1))) + Vec3<float>(globalCOMv_leg->block(0,1,3,1)).dot(Vec3<float>(globalCOMv_leg->block(0,1,3,1)))*quadruped._hipInertia.getMass();
    _KinEnergyLeg.operator()(i) += Vec3<float>(globalCOMw_leg->block(0,2,3,1)).dot(quadruped._kneeInertia.getInertiaTensor()*Vec3<float>(globalCOMw_leg->block(0,2,3,1))) + Vec3<float>(globalCOMv_leg->block(0,2,3,1)).dot(Vec3<float>(globalCOMv_leg->block(0,2,3,1)))*quadruped._kneeInertia.getMass();

    // Sum of the potential rot and lin energy for all leg bodies
    _PotEnergyLeg.operator()(i) = globalCOMp_leg->operator()(2,0)*quadruped._abadInertia.getMass();
    _PotEnergyLeg.operator()(i) += globalCOMp_leg->operator()(2,1)*quadruped._hipInertia.getMass();
    _PotEnergyLeg.operator()(i) += globalCOMp_leg->operator()(2,2)*quadruped._kneeInertia.getMass();
  }
  _KinEnergyLeg = float(0.5) * _KinEnergyLeg;
  _PotEnergyLeg = g[2] * _PotEnergyLeg;
  // _state_coord = this->_data->._legController->datas.p;
  // std::cout << "Kinetic Energy" << _KinLinEnergy << "\n";
  // Vec4<float> a(_KinLinEnergy +_KinRotEnergy + _PotEnergy, 0, 0, 0);
  return _KinEnergyLeg + _PotEnergyLeg;
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
