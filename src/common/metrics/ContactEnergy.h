#pragma once

#include "Metric.h"

using Eigen::Array4f;
using Eigen::Array4i;

// template <typename T>
class ContactEnergy : public Metric 
{
public:
  ContactEnergy();
  ~ContactEnergy() = default;

  Vec4<float> getFinalBodyCost();
  Vec4<float> getFinalCost();
  Vec4<float> getFinalLegCost();

  // void setRobotData(ControlFSMData<T>& data);
  // void debugPrint();
//   void earlyContactHandle(Vec4<uint8_t> , int , int ) {}
//   void restoreDefaults(){}
  Vec3<float> g;

private:
  Eigen::Matrix<float, 18, 18> Total_mass_matrix;
  Eigen::Matrix<float, 24, 18> Total_Jacobi;
  Eigen::Matrix<float, 24, 12> Contact_matrix;
  Eigen::Matrix<float, 6, 3> Leg_Jacobi;
  Vec4<float> _test;
  Vec3<float> _vBody;
  Vec3<float> _position;
  Vec3<float> _state_coord;
  LegControllerData<float> datas[4];
  float _KinLinEnergy;
  float _KinRotEnergy;
  Vec4<float> _KinEnergyLeg;
  float _PotEnergy;
  Vec4<float> _PotEnergyLeg;
};
