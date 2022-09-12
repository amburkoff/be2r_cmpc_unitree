#include "Metric.h"

// template <typename T>
void Metric::setRobotData(ControlFSMData<float> &data)
{
  _data = &data;
}
void Metric::debugPrint()
{
  _data->debug->metric_data.final_body_cost = getFinalBodyCost();
  _data->debug->metric_data.final_leg_cost = getFinalLegCost();
  _data->debug->metric_data.final_cost = getFinalCost();
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
    leg_w->block(0,2,3,1) = matToSkewVec(w1) + R1*matToSkewVec(w2 + w3); 
    // The second rotation doesn't affect on the third axis of rotation 
    //so that's why [w1] + R1[w2]R1^T + R1[w3]R1^T
    // the equation above is equal to w1 + R1*(w2 + w3) 
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