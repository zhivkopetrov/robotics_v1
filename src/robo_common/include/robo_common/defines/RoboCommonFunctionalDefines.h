#ifndef ROBO_COMMON_ROBOCOMMONFUNCTIONALDEFINES_H_
#define ROBO_COMMON_ROBOCOMMONFUNCTIONALDEFINES_H_

//System headers
#include <functional>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/field/FieldPos.h"
#include "utils/drawing/Point.h"

//Forward declarations

using PlayerDamageCb = std::function<void(int32_t)>;
using SetFieldDataMarkerCb = std::function<void(const FieldPos&, char)>;
using ResetFieldDataMarkerCb = std::function<void(const FieldPos&)>;
using GetFieldDescriptionCb = std::function<const FieldDescription&()>;
using GetIndicatorPanelValue = std::function<int32_t()>;
using RobotActCb = std::function<void(MoveType)>;
using GetRobotStateCb = std::function<RobotState()>;
using GetRobotAbsolutePosCb = std::function<Point()>;
using GetRobotRotationAngleCb = std::function<double()>;
using GetPlayerSurroundingTilesCb = std::function<SurroundingTiles()>;
using FinishRobotActCb = std::function<void(const RobotState&, MoveOutcome)>;
using CancelRobotMove = std::function<void()>;

//invoked before the collision animation has been started
using ToggleDebugInfoCb = std::function<void()>;
using ToggleHelpPageCb = std::function<void()>;
using PlayerRobotDamageCollisionCb = std::function<void()>;
using IndicatorDepletedCb = std::function<void()>;
using NumberCounterTargetReachedCb = std::function<void()>;
using ShutdownGameCb = NumberCounterTargetReachedCb;
using StartGameLostAnimCb = IndicatorDepletedCb;
using StartGameWonAnimCb = std::function<void()>;
using StartAchievementWonAnimCb = std::function<void(Achievement)>;
using RevealFogOfWarTilesCb = std::function<void()>;
using ObjechApproachOverlayTriggeredCb = std::function<void(const FieldPos&)>;
using ContainerRedrawCb = std::function<void()>;
using TileReleavedCb = std::function<void()>;

#endif /* ROBO_COMMON_ROBOCOMMONFUNCTIONALDEFINES_H_ */
