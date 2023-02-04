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
  void runInternalThread();
  void invokeSingleCommand(const MotionCommand& cmd);

  MotionSequenceExecutorOutInterface _outInterface;
  std::mutex _actionDoneMutex;
  MotionActionDoneCb _actionDoneCb;

  std::thread _thread;
  ThreadSafeQueue<MotionCommand> _commandQueue;
};

#endif /* UR_CONTROL_COMMON_MOTIONSEQUENCEEXECUTOR_H_ */
