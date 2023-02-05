#ifndef UR_CONTROL_COMMON_MOTIONSEQUENCEEXECUTOR_H_
#define UR_CONTROL_COMMON_MOTIONSEQUENCEEXECUTOR_H_

//System headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"
#include "utils/concurrency/ThreadSafeQueue.h"
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

struct MotionSequenceExecutorOutInterface {
  PublishURScriptCb publishURScriptCb;
  InvokeURScriptServiceCb invokeURScriptServiceCb; 
};

class MotionSequenceExecutor : public NonCopyable, public NonMoveable { 
public:
  ErrorCode init(const MotionSequenceExecutorOutInterface& outInterface);

  void shutdown();

  void dispatchAsync(const std::vector<MotionCommand>& commands, 
                     const MotionActionDoneCb& actionDoneCb);

private:
  using BlockingTask = std::function<void()>;

  void runCommandComsumerLoop();
  void invokeSingleCommand(const MotionCommand& cmd);

  void runBlockingInvokerLoop();

  MotionSequenceExecutorOutInterface _outInterface;

  std::mutex _actionDoneMutex;
  MotionActionDoneCb _actionDoneCb;
  std::atomic<bool> _preemptCurrCommand = false;

  std::thread _commandConsumerThread;
  ThreadSafeQueue<MotionCommand> _commandQueue;

  //performs blocking tasks such as invoking ROS2 services
  std::thread _blockingInvokerThread;
  ThreadSafeQueue<BlockingTask> _blockingTasksQueue;
};

#endif /* UR_CONTROL_COMMON_MOTIONSEQUENCEEXECUTOR_H_ */
