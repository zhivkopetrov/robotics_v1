#ifndef ROBO_COMMON_ROBOCOMMONFUNCTIONALDEFINES_H_
#define ROBO_COMMON_ROBOCOMMONFUNCTIONALDEFINES_H_

//System headers
#include <functional>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/FieldPos.h"

//Forward declarations

using PlayerDamageCb = std::function<void(int32_t)>;
using SetFieldDataMarkerCb = std::function<void(const FieldPos&, char)>;
using ResetFieldDataMarkerCb = std::function<void(const FieldPos&)>;
using GetFieldDescriptionCb = std::function<const FieldDescription&()>;
using RobotActCb = std::function<void(MoveType)>;
using GetRobotStateCb = std::function<RobotState()>;
using GetPlayerSurroundingTilesCb = std::function<SurroundingTiles()>;
using FinishRobotActCb = std::function<void(const RobotState&, MoveOutcome)>;
using IndicatorDepletedCb = std::function<void()>;
using NumberCounterTargetReachedCb = std::function<void()>;
using ShutdownGameCb = NumberCounterTargetReachedCb;
using StartGameLostAnimCb = IndicatorDepletedCb;
using StartGameWonAnimCb = std::function<void()>;
using StartAchievementWonAnimCb = std::function<void(Achievement)>;

#endif /* ROBO_COMMON_ROBOCOMMONFUNCTIONALDEFINES_H_ */
