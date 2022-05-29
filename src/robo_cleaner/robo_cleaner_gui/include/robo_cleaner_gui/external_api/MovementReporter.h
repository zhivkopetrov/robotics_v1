#ifndef ROBO_CLEANER_GUI_MOVEMENTREPORTER_H_
#define ROBO_CLEANER_GUI_MOVEMENTREPORTER_H_

//System headers
#include <thread>

//Other libraries headers

#include "utils/concurrency/ThreadSafeQueue.h"
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_cleaner_gui/defines/RoboCleanerGuiFunctionalDefines.h"

//Forward declarations

using ResetControllerStatusCb = std::function<void()>;

class MovementReporter: public NonCopyable, public NonMoveable {
public:
  ErrorCode init(const ResetControllerStatusCb& resetControllerStatusCb);

  void deinit();

  void acceptGoal(const std::shared_ptr<GoalHandleRobotMove> &goalHandle);

  void reportProgress(const MoveProgress &progress);

private:
  void run();

  void reportProgressLoop(
      const std::shared_ptr<GoalHandleRobotMove> &goalHandle);

  std::thread _thread;
  ThreadSafeQueue<std::shared_ptr<GoalHandleRobotMove>> _activeGoals;
  ThreadSafeQueue<MoveProgress> _progressReports;

  ResetControllerStatusCb _resetControllerStatusCb;
};

#endif /* ROBO_CLEANER_GUI_MOVEMENTREPORTER_H_ */
