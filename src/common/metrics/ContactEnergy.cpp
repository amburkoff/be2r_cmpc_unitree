#include "ContactEnergy.h"



ContactEnergy::ContactEnergy()
{
  _Total_mass_matrix.setZero();
  _Leg_mass_matrix.setZero();
  _Total_Jacobi_body.setZero();
  _Total_Jacobi_world.setZero();
  _Contact_matrix.setZero();
  _Leg_Jacobi.setZero();

  _Total_Jacobi_body.block(0,0,6,6) = Eigen::Matrix<float, 6, 6>::Identity();
  //Total_Jacobi_body.block(3,0,3,3) = Eigen::Matrix<float, 3, 3>::Identity();
  //Total_Jacobi_body.block(6,0,3,3) = Eigen::Matrix<float, 3, 3>::Identity();
  //Total_Jacobi_body_body.block(9,0,3,3) = Eigen::Matrix<float, 3, 3>::Identity();
  // g = Vec3<float>(0, 0, -9.81);
  // _KinLinEnergy = float(0);
  // _KinRotEnergy = float(0);
  // _PotEnergy = float(0);
  // _KinEnergyLeg.setZero();
  // _PotEnergyLeg.setZero();
};

Vec4<float> ContactEnergy::getFinalCost()
{
  
  return Vec4<float>(0,0,0,0);
}

Vec4<float> ContactEnergy::getFinalBodyCost()
{
//   _vBody = this->_data->_stateEstimator->getResult().vBody;
//   _KinLinEnergy = _vBody.dot(_vBody) * this->_data->_quadruped->_bodyMass * float(0.5);
//   _position = this->_data->_stateEstimator->getResult().position;
//   _PotEnergy = this->_data->_quadruped->_bodyMass * _position[2] * g[2];
//   Vec4<float> Result;
//   _KinRotEnergy = float(0.5)*this->_data->_stateEstimator->getResult().omegaBody.dot(this->_data->_quadruped->_bodyInertia.getInertiaTensor()*this->_data->_stateEstimator->getResult().omegaBody);
//   Result.setZero();
//   Result.operator()(0) = _KinLinEnergy;
//   Result.operator()(1) = _KinRotEnergy;
//   Result.operator()(2) = _PotEnergy;
//   Result.operator()(3) = _KinLinEnergy + _KinRotEnergy + _PotEnergy;
return Vec4<float>(0,0,0,0);
}

/////////////////////////////////////Math review////////////////////////////////////
// M(q(t))ddq(t) + C(q(t),dq(t))+G(q(t)) = tau + J(q(t))^T*F (1)
// q -- [q_i,x,y,z]; q_i --joint angles ; x,y,z - body postion
// F -- reaction force to the end-effector
// Let's asume that for the small dt that calculates from "t+"-"t-"=dt 
// so
// q(t+)= q(t-) and tau(t+)=tau(t-) (2)
// but dq(t+) <> dq(t-) ddq(t+) <> ddq(t-) (3)

//let's calculate impulse change rule from equation (1)
//  M(q(t+))*dq(t+)-M(q(t-))*dq(t-) + integarl[G(q(t+))] - integarl[G(q(t-))] = 
// = integarl[tau(t+)]-integarl[tau(t-)] + integarl[J(q(t+))^T*F(t+)] - integarl[J(q(t+))^T*F(t-)] (4)

// Using eq.(2-3) and facts that:
//   1) before the contact F(t-) == 0 
//   2) after contact happend J(q(t+))*dq(t+)=0

// So (4) becames M(q(t+))*dq(t+)-M(q(t+))*dq(t-) = J(q(t+))^T*integarl[F(t+)] (5)
// From (5) we can find dq(t+) and integarl[F(t+)] as
// integarl[F(t+)] = -(J(q)^T/M(q)*J(q))\J(q)^T*dq(t-) (6)
// dq(t+) = (I-M\J/(J^T/M*J)*J^T)*dq(t-) (7)
// After that let's compute energy change dE = 1/2*(dq(t+)^T*M(q(t+))*dq(t+) - dq(t-)^T*M(q(t-))*dq(t-))
// as dE = -1/2*dq(t-)^T*(J/(J^T/M*J)*J^T*dq(t-)) (8)
////////////////////////////////////////////////////////////////////////////////
// _s -- space frame, _b -- body frame, _l -- leg shoulder frame, _f -- leg contact frame (feet)
// U_sf -- velocity of feet frame relative to space frame
// [w_sb] - skew-symmetry matrix from rotational velocity of body frame relative to space frame
// p_bl -- distance from body frame to leg frame

// Let's calculate Twist_sf from Twist_sb = [w_sb U_sb]' and Twist_lf = [w_lf; U_lf] - 6x1
// Twist_sf = Twist_sb + Ad[T_sb]*Ad[T_bl]*Twist_lf , R_bl = I(3)
// where Ad[T_bl] = [R_bl 0; [p_bl]*R_bl] -- adjoint matrix 6x6 for the homogeneous
// transformation T_bl from body to leg frame
// Twist_sf - Twist_sb = [R_sb*w_lf; [p_sb]*R_sb*w_lf + R_sb*([p_bl]*w_lf + U_lf]
// Twist_23 = [w_3; -[p_3]*R_3*w_3]
// w_lf = w_1 + R_1*(w_2 + R_2*w_3), where 1,2,3 -- leg joints from shoulder to feet
// U_lf = [p_1]*R_1*(w_2 + R_2*w3) + [p_2]*R_2*w_3

// U_sf = -([w_sb] + [R_sb*w_lf])*p_sb + R_sb*[w_lf]*(p_lf + p_bl) + R_sb*dp_lf + dp_sb
// P_kin(t) = M_b*Twist_sb + Sum(for legs that are not in contact)sum(i=1..3)(M_i*Twist_sli)= M(q,U_b,w_b)*[q,U_b,w_b]
// 
////////////////////////////////////////////////////////////////////////////////

Vec4<float> ContactEnergy::getFinalLegCost()
{
  // Eigen::Matrix<float, 6, 3> J_v;
  Mat3<float> _Rbod = this->_data->_stateEstimator->getResult().rBody.transpose();
  Vec6<float> _Twist_W;
  _Twist_W.block(0,0,3,1) = this->_data->_stateEstimator->getResult().omegaWorld;
  _Twist_W.block(3,0,3,1) = this->_data->_stateEstimator->getResult().vWorld;
  // Mat3<float> Jacobi;
  // Mat3<float> globalCOMp_leg;
  // Mat3<float> globalCOMv_leg;
  // Mat3<float> globalCOMw_leg;
  // Jacobi.setZero();
  // globalCOMp_leg.setZero();
  // globalCOMv_leg.setZero();
  // globalCOMw_leg.setZero();
  // _KinEnergyLeg.setZero();
  // _PotEnergyLeg.setZero();
  // J_v.block(0,0,3,3) = coordinateRotation(CoordinateAxis::Y,float(0));
  // J_v.block(3,3,3,3) = coordinateRotation(CoordinateAxis::Y,float(0));
  // _Leg_Jacobi.block(0,0,6,1) = J_v.block(0,1,6,1);
  Vec4<uint8_t> contactState = _data->_stateEstimator->getContactSensorData();
  Vec4<float> dE(0,0,0,0);
  for (int i = 0; i < 4; i++)
  {
    if (contactState[i] == uint8_t(1))
      {
      Quadruped<float> &quadruped = *this->_data->_quadruped;
      Vec6<float> _S1;
      Vec6<float> _S2;
      Vec6<float> _S3;
      _S1.setZero();
      _S2.setZero();
      _S3.setZero();

      _S1.block(0,0,3,1)= Vec3<float>(1,0,0);
      _S2.block(0,0,3,1)= Vec3<float>(0,1,0);
      _S3.block(0,0,3,1)= Vec3<float>(0,1,0);
      Mat3<float> _w1 = vectorToSkewMat(Vec3<float>(1,0,0));//_S1.block(0,0,3,1));
      Mat3<float> _w2 = vectorToSkewMat(Vec3<float>(0,1,0));//_S2.block(0,0,3,1));
      Mat3<float> _w3 = vectorToSkewMat(Vec3<float>(0,1,0));//_S3.block(0,0,3,1));
      Vec3<float> _r1(0,0,-quadruped._abadLinkLength);
      Vec3<float> _r2(0,0,-quadruped._hipLinkLength); //wrong vector. correct it by latex
      Vec3<float> _r3(0,0,-quadruped._kneeLinkLength);
      
      _S1.block(3,0,3,1) = -_w1*_r1;
      _S2.block(3,0,3,1) = -_w2*_r2;
      _S3.block(3,0,3,1) = -_w3*_r3;
      Vec3<float> q = _data->_legController->datas[i].q;
      Mat3<float> _R1 = Eigen::Matrix<float, 3, 3>::Identity() + _w1*sin(q[0]) + _w1*_w1*(1-cos(q[0]));
      Vec3<float> _p1 = (Eigen::Matrix<float, 3, 3>::Identity()*q[0] + _w1*(1-cos(q[0])) - _w1*_w1*sin(q[0]))*_S1.block(3,0,3,1);
      Mat3<float> _R2 = Eigen::Matrix<float, 3, 3>::Identity() + _w2*sin(q[1]) + _w2*_w2*(1-cos(q[1]));
      Vec3<float> _p2 = (Eigen::Matrix<float, 3, 3>::Identity()*q[1] + _w2*(1-cos(q[1])) - _w2*_w2*sin(q[1]))*_S2.block(3,0,3,1);
      Mat3<float> _R3 = Eigen::Matrix<float, 3, 3>::Identity() + _w3*sin(q[2]) + _w3*_w3*(1-cos(q[2]));
      Vec3<float> _p3 = (Eigen::Matrix<float, 3, 3>::Identity()*q[2] + _w3*(1-cos(q[2])) - _w3*_w3*sin(q[2]))*_S3.block(3,0,3,1);
    
      _Leg_Jacobi.block(0,0,6,1) = _S1;//coordinateRotation(CoordinateAxis::X,q[0]);
      _Leg_mass_matrix = _Leg_Jacobi.transpose()*quadruped._abadInertia.getMatrix()*_Leg_Jacobi;
      _Leg_Jacobi.block(0,1,6,1) = createSXform(_R1,_p1)*_S2; // coordinateRotation(CoordinateAxis::Y,q[1])*coordinateRotation(CoordinateAxis::Y,q[1]);
      _Leg_mass_matrix += _Leg_Jacobi.transpose()*quadruped._hipInertia.getMatrix()*_Leg_Jacobi;
      _Leg_Jacobi.block(0,2,6,1) = createSXform(_R1*_R2,_R1*_p2 + _p1)*_S3;
      _Leg_mass_matrix += _Leg_Jacobi.transpose()*quadruped._kneeInertia.getMatrix()*_Leg_Jacobi;

      Vec3<float> _ph = quadruped.getHipLocation(i); // hip positions relative to CoM
      // compute velocities and positions of the leg COM in the local fixed hip frame 
      // // Metric::computeCenterLegVelAndPos(quadruped,this->_data->_legController->datas[i].q,this->_data->_legController->datas[i].qd,&(Jacobi),&(globalCOMp_leg),&(globalCOMv_leg),&(globalCOMw_leg),i);
      _Total_Jacobi_body.block(0,6,6,3) = createSXform(Eigen::Matrix<float, 3, 3>::Identity(),_ph)*_Leg_Jacobi;
      Vec3<float> _pbod = this->_data->_stateEstimator->getResult().position;
      _Total_Jacobi_world.block(0,6,6,3) = createSXform(_Rbod,_pbod)*_Total_Jacobi_body.block(0,6,6,3);
      _Total_Jacobi_world.block(0,0,6,6) = Eigen::Matrix<float,6,6>::Identity();

      _Total_mass_matrix.block(0,0,6,6) = quadruped._bodyInertia.getMatrix();
      _Total_mass_matrix.block(6,6,3,3) = _Leg_mass_matrix;
      // // Total_mass_matrix.block(3,3,3,3) = 
      // // Body COM velocity and position in the global frame
      _position = this->_data->_stateEstimator->getResult().position;
      _vBody = this->_data->_stateEstimator->getResult().vBody;
      Eigen::Matrix<float, 9, 1> qV;
      qV.block(0,0,6,1) = _Twist_W;
      qV(6,0) = this->_data->_legController->datas[i].qd[0];
      qV(7,0) = this->_data->_legController->datas[i].qd[1];
      qV(8,0) = this->_data->_legController->datas[i].qd[2];
      // // as dE = -1/2*dq(t-)^T*(J/(J^T/M*J)*J^T*dq(t-)) (8)
      Vec6<float> qV_extr;
      qV_extr= _Total_Jacobi_world*qV;
      Eigen::Matrix<float, 6, 1> eqV;
      eqV.setZero();
      Eigen::Matrix<float, 9, 6> JT;
      JT = _Total_Jacobi_world.transpose();
      Mat6<float> Extr_Mass;
      Extr_Mass = Eigen::Matrix<float,6,6>::Identity();
      Extr_Mass = _Total_Jacobi_world*_Total_mass_matrix*JT;
      eqV = Extr_Mass.lu().solve(qV_extr);
      dE[i] = qV_extr.dot(eqV);
  // //     Vec3<float> p_rel = _ph + this->_data->_legController->datas[i].p; // Local frame distance from COM to leg(i)
      
  // //     // Local frame velocity of leg(i) relative to COM
  // //     Vec3<float> dp_rel = this->_data->_legController->datas[i].v;

  // //     // Distance to leg(i) from Aligned with World frame body frame
  // //     Vec3<float> p_f = _Rbod * p_rel;

  // //     // World leg(i) velocity in Aligned with World frame body frame
  // //     Vec3<float> dp_f = _Rbod * (this->_data->_stateEstimator->getResult().omegaBody.cross(p_rel) + dp_rel);

  // //     // std::cout<< globalCOMp_leg.block(0,0,1,1)<< endl;
  // //     // Move from i-th leg hip local frame to the body frame
  // //     globalCOMp_leg.block(0,0,3,1) += _ph;//globalCOMp_leg.block(0,0,3,1) + 
  // //     globalCOMp_leg.block(0,1,3,1) += _ph;
  // //     globalCOMp_leg.block(0,2,3,1) += _ph;
      
  // //     // added linear velocity component from body rotations
  // //     globalCOMv_leg.block(0,0,3,1) += this->_data->_stateEstimator->getResult().omegaBody.cross(Vec3<float>(globalCOMp_leg.block(0,0,3,1)));
  // //     globalCOMv_leg.block(0,1,3,1) += this->_data->_stateEstimator->getResult().omegaBody.cross(Vec3<float>(globalCOMp_leg.block(0,1,3,1)));
  // //     globalCOMv_leg.block(0,2,3,1) += this->_data->_stateEstimator->getResult().omegaBody.cross(Vec3<float>(globalCOMp_leg.block(0,2,3,1)));
      
  // //     // Added body angular velocity
  // //     globalCOMw_leg.block(0,0,3,1) = globalCOMw_leg.block(0,0,3,1) + this->_data->_stateEstimator->getResult().omegaBody;
  // //     globalCOMw_leg.block(0,1,3,1) = globalCOMw_leg.block(0,1,3,1) + this->_data->_stateEstimator->getResult().omegaBody;
  // //     globalCOMw_leg.block(0,2,3,1) = globalCOMw_leg.block(0,2,3,1) + this->_data->_stateEstimator->getResult().omegaBody;
      
  // //     // Rotated, World leg(i) COM positions and velocities are expressed 
  // //     // // in Aligned with World frame body frame
  // //     globalCOMv_leg.block(0,0,3,3) = _Rbod * globalCOMv_leg;
  // //     globalCOMp_leg.block(0,0,3,3) = _Rbod * globalCOMp_leg;
  // //     globalCOMw_leg.block(0,0,3,3) = _Rbod * globalCOMw_leg;
      
  // //     // Expressed in the global frame
  // //     globalCOMv_leg.block(0,0,3,1) += _vBody;
  // //     globalCOMv_leg.block(0,1,3,1) += _vBody;
  // //     globalCOMv_leg.block(0,2,3,1) += _vBody;

  // //     // // Global position of the COM for the leg Expressed in the world frame (initial frame)
  // //     globalCOMp_leg.block(0,0,3,1) += _position;
  // //     globalCOMp_leg.block(0,1,3,1) += _position;
  // //     globalCOMp_leg.block(0,2,3,1) += _position;

  // //     // Sum of the kinetic rotational and linear energy for all leg bodies
  // //     _KinEnergyLeg.operator()(i) = Vec3<float>(globalCOMw_leg.block(0,0,3,1)).dot(quadruped._abadInertia.getInertiaTensor()*Vec3<float>(globalCOMw_leg.block(0,0,3,1))) + Vec3<float>(globalCOMv_leg.block(0,0,3,1)).dot(Vec3<float>(globalCOMv_leg.block(0,0,3,1)))*quadruped._abadInertia.getMass();
  // //     _KinEnergyLeg.operator()(i) += Vec3<float>(globalCOMw_leg.block(0,1,3,1)).dot(quadruped._hipInertia.getInertiaTensor()*Vec3<float>(globalCOMw_leg.block(0,1,3,1))) + Vec3<float>(globalCOMv_leg.block(0,1,3,1)).dot(Vec3<float>(globalCOMv_leg.block(0,1,3,1)))*quadruped._hipInertia.getMass();
  // //     _KinEnergyLeg.operator()(i) += Vec3<float>(globalCOMw_leg.block(0,2,3,1)).dot(quadruped._kneeInertia.getInertiaTensor()*Vec3<float>(globalCOMw_leg.block(0,2,3,1))) + Vec3<float>(globalCOMv_leg.block(0,2,3,1)).dot(Vec3<float>(globalCOMv_leg.block(0,2,3,1)))*quadruped._kneeInertia.getMass();

  // //     // Sum of the potential rot and lin energy for all leg bodies
  // //     _PotEnergyLeg.operator()(i) = globalCOMp_leg.operator()(2,0)*quadruped._abadInertia.getMass();
  // //     _PotEnergyLeg.operator()(i) += globalCOMp_leg.operator()(2,1)*quadruped._hipInertia.getMass();
  // //     _PotEnergyLeg.operator()(i) += globalCOMp_leg.operator()(2,2)*quadruped._kneeInertia.getMass();
      }
      else
      {
        dE[i] = _data->debug->metric_data.final_leg_cost[i];
      }
  }
  // _KinEnergyLeg = float(0.5) * _KinEnergyLeg;
  // _PotEnergyLeg = g[2] * _PotEnergyLeg;
  // _state_coord = this->_data->._legController->datas.p;
  // std::cout << "Kinetic Energy" << _KinLinEnergy << "\n";
  //  a(_KinLinEnergy +_KinRotEnergy + _PotEnergy, 0, 0, 0);
  return dE;
};