#ifndef UR_CONTROL_COMMON_MOTIONSEQUENCEEXECUTOR_H_
#define UR_CONTROL_COMMON_MOTIONSEQUENCEEXECUTOR_H_

//System headers
#include <cstdint>
#include <vector>
#include <atomic>
#include <thread>

//Other libraries headers
#include "ur_control_common/defines/UrControlCommonFunctionalDefines.h"
#include "game_engine/defines/ActionEventDefines.h"
#include "utils/concurrency/ThreadSafeQueue.h"
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

struct MotionSequenceExecutorOutInterface {
  InvokeActionEventCb invokeActionEventCb;
  PublishURScriptCb publishURScriptCb;
  InvokeURScriptServiceCb invokeURScriptServiceCb;
  InvokeURScriptPreemptServiceCb invokeURScriptPreemptServiceCb;
};

class MotionSequenceExecutor : public NonCopyable, public NonMoveable { 
public:
  ErrorCode init(const MotionSequenceExecutorOutInterface& outInterface);

  void shutdown();

  //NOTE: actionDoneCb will be called through the ActionEventSystem
  //      with ActionEventType::NON_BLOCKING param
  void dispatchUscriptsAsync(const std::vector<UrscriptCommand>& commands, 
                             const UrscriptsBatchDoneCb& batchDoneCb);

private:
  using BlockingTask = std::function<void()>;

  void runCommandComsumerLoop();
  void runBlockingInvokerLoop();
  
  void invokeCommand(const UrscriptCommand& cmd);
  void invokeNonBlockingCommand(const UrscriptCommand& cmd);
  void invokeBlockingCommand(const UrscriptCommand& cmd);

  MotionSequenceExecutorOutInterface _outInterface;

  std::mutex _urscriptsBatchDoneMutex;
  UrscriptsBatchDoneCb _urscriptsBatchDoneCb;
  std::atomic<bool> _preemptCurrCommand = false;

  std::thread _commandConsumerThread;
  ThreadSafeQueue<UrscriptCommand> _commandQueue;

  //performs blocking tasks such as invoking ROS2 services
  std::thread _blockingInvokerThread;
  ThreadSafeQueue<BlockingTask> _blockingTasksQueue;
};

#endif /* UR_CONTROL_COMMON_MOTIONSEQUENCEEXECUTOR_H_ */
