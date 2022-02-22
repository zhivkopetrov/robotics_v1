#ifndef ROBO_COMMON_ROBOCOMMONFUNCTIONALDEFINES_H_
#define ROBO_COMMON_ROBOCOMMONFUNCTIONALDEFINES_H_

//C system headers

//C++ system headers
#include <functional>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/FieldPos.h"

//Forward declarations

using PlayerDamageCb = std::function<void(int32_t)>;
using SetFieldDataMarkerCb = std::function<void(const FieldPos&, char)>;
using ResetFieldDataMarkerCb = std::function<void(const FieldPos&)>;
using GetFieldDataCb = std::function<const FieldData&()>;
using RobotActCb = std::function<void(MoveType)>;
using GetRobotFieldPosCb = std::function<FieldPos()>;
using GetRobotDirCb = std::function<Direction()>;
using FinishRobotActCb = std::function<void(int32_t)>;
using IndicatorDepletedCb = std::function<void()>;
using NumberCounterTargetReachedCb = std::function<void()>;
using ShutdownGameCb = NumberCounterTargetReachedCb;
using StartGameLostAnimCb = IndicatorDepletedCb;
using StartGameWonAnimCb = std::function<void()>;
using StartAchievementWonAnimCb = std::function<void(Achievement)>;

#endif /* ROBO_COMMON_ROBOCOMMONFUNCTIONALDEFINES_H_ */
