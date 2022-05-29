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

class MovementWatcher: public NonCopyable, public NonMoveable {
public:
  ErrorCode init(const ReportMoveProgressCb& reportMoveProgressCb);

  void process();

  void changeState(const RobotState& state, MoveOutcome outcome);

private:
  ReportMoveProgressCb _reportMoveProgressCb;
  MoveProgress _currProgress;
};

#endif /* ROBO_CLEANER_GUI_MOVEMENTWATCHER_H_ */
