// #include "Metrics.h"

// void ContactEnergy()
// {

// }
// void ~ContactEnergy()
// {
  
// }
// void SystemEnergy()
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

// void MixedFrequncyGait::debugPrint() {}

// void OffsetDurationGait::earlyContactHandle(Vec4<uint8_t> footSensorState, int iterationsBetweenMPC,
//                                             int currentIteration)
// {
//   for (long leg = 0; leg < 4; leg++)
//   {
//     // Если ранний контакт обнаружен в заключительной части свинг фазы
//     if ((getSwingState()[leg] > 0.75f) && (footSensorState(leg) == 1))
//     {
//       // Уменьшить оффсет, увеличить duration на ту же величину
//       std::cout << "SWING STATE before" << getSwingState()[leg] << std::endl;
//       float difference = _offsetsFloat(leg) - _phase < -0.001f ? _offsetsFloat(leg) - _phase + 1.0f
//                                                                : _offsetsFloat(leg) - _phase;
//       std::cout << "difference in phase " << difference << std::endl;
//       std::cout << "phase" << _phase << std::endl;
//       std::cout << "offset [ " << leg << "] " << _offsetsFloat(leg) << std::endl;
//       std::cout << "duration before [ " << leg << "] " << _durationsFloat(leg) << std::endl;
//       _offsetsFloat(leg) -= difference * 1.0001;
//       //      _offsets(leg) = int(_offsetsFloat(leg)) * _nIterations;
//       //      if (_offsetsFloat(leg) == 0.5f)
//       //        _durationsFloat(leg) += difference;
//       //      if (_offsetsFloat(leg) <= 0.01f)
//       _durationsFloat(leg) += difference;

//       _durations(leg) = int(_durationsFloat(leg) * float(_nIterations));
//       _offsets(leg) = int(_offsetsFloat(leg) * float(_nIterations));
//       //      std::cout << "_durations(leg)" << _durations(leg) << std::endl;
//       std::cout << "offset after [ " << leg << "] " << _offsetsFloat(leg) << std::endl;
//       std::cout << "duration after [ " << leg << "] " << _durationsFloat(leg) << std::endl;
//       std::cout << "SWING STATE after" << getSwingState()[leg] << std::endl;
//       std::cout << "STANCE STATE after" << getContactState()[leg] << std::endl;
//     }
//     if ((getSwingState()[leg] < 0.25f) && (footSensorState(leg) == 1))
//     {
//     }
//   }
// }

// void OffsetDurationGait::restoreDefaults()
// {
//   _durationsFloat = _durationsF_defaults;
//   _durations = _durations_defaults;
//   _offsets = _offsets_defaults;
//   _offsetsFloat = _offsetsF_defaults;
// }
