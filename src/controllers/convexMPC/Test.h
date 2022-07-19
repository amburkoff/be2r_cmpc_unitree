// #ifndef PROJECT_METRICS_H
// #define PROJECT_METRICS_H

// #include <queue>
// #include <string>
// #include <iostream>
// #include "cppTypes.h"
// #include <cmath>
// #include <ControlFSMData.h>
// // #include "LegController.h"

// // template <typename T>
// class Metric
// {
// public:
//   virtual ~Metric() = default;
//   virtual Vec4<float> getFinalLegCost() = 0;
//   // ControlFSMData<T>* _data;
  
// };

// using Eigen::Array4f;
// using Eigen::Array4i;


// class SystemEnergy : public Metric
// {
// public:
//   SystemEnergy();
//   ~SystemEnergy() = default;

//   Vec4<float> getFinalLegCost();

//   // void setRobotData(ControlFSMData<T>& data);
//   void debugPrint();
// //   void earlyContactHandle(Vec4<uint8_t> , int , int ) {}
// //   void restoreDefaults(){}

// private:
//   Vec4<float> _test;
//   Vec3<float> _vBody;
//   Vec3<float> _position;
//   Vec3<float> _state_coord;
//   LegControllerData<float> datas[4];
//   float _KinEnergy;
//   float _PotEnergy;
// };

// #endif //PROJECT_METRICS_H