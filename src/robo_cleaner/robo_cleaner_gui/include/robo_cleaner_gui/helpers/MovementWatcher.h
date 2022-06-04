#ifndef ROBO_CLEANER_GUI_MOVEMENTWATCHER_H_
#define ROBO_CLEANER_GUI_MOVEMENTWATCHER_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiFunctionalDefines.h"

//Forward declarations

struct MovementWatcherOutInterface {
  ReportMoveProgressCb reportMoveProgressCb;
  GetRobotStateCb getRobotStateCb;
  GetRobotAbsolutePosCb getRobotAbsolutePosCb;
  GetRobotRotationAngleCb getRobotRotationAngleCb;
  GetFieldDescriptionCb getFieldDescriptionCb;
};

struct MovementWatcherConfig {
  int32_t tileWidth{};
  int32_t tileHeight{};
};

class MovementWatcher: public NonCopyable, public NonMoveable {
public:
  ErrorCode init(MovementWatcherConfig& cfg,
                 const MovementWatcherOutInterface& interface);

  void process();

  void changeState(const RobotState& state, MoveOutcome outcome);

  void onRobotStartingAct(MoveType moveType, char approachingMarker);

  void onObstacleApproachTrigger(const FieldPos& fieldPos);

  void cancelFeedbackReporting();

private:
  ErrorCode initOutInterface(const MovementWatcherOutInterface& interface);

  void processForwardMovement();
  void processRotationMovement();

  void reset();

  MovementWatcherOutInterface _outInterface;

  MoveProgress _currProgress;

  MovementWatcherConfig _cfg;
  MoveType _currMoveType = MoveType::UNKNOWN;
  RotationDir _currRotationDir = RotationDir::LEFT;

  Point _targetAbsolutePos;
  double _targetRotation{};

  bool _isFeedbackReportActive = false;
};

#endif /* ROBO_CLEANER_GUI_MOVEMENTWATCHER_H_ */
