#ifndef UR_CONTROL_COMMON_MOTIONSEXECUTOR_H_
#define UR_CONTROL_COMMON_MOTIONSEXECUTOR_H_

//System headers
#include <cstdint>
#include <unordered_map>
#include <memory>

//Other libraries headers
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "ur_control_common/motion/MotionSequence.h"
#include "ur_control_common/motion/MotionSequenceExecutor.h"

//Forward declarations

enum class MotionAction {
  START, GRACEFUL_STOP, ABORT, RECOVER
};

class MotionExecutor : public NonCopyable, public NonMoveable { 
public:
  ErrorCode init(const MotionSequenceExecutorOutInterface& outInterface);
  ErrorCode addSequence(std::unique_ptr<MotionSequence> sequence);
  ErrorCode loadSequence(int32_t id);
  ErrorCode performAction(
    MotionAction action, const MotionCommandBatchDoneCb& doneCb);

  void shutdown();

private:
  enum InternalDefines {
    EMPTY_SEQUENCE_ID = -1
  };

  std::unordered_map<int32_t, std::unique_ptr<MotionSequence>> 
    _supportedSequences;
  int32_t _currSequenceId = EMPTY_SEQUENCE_ID;

  MotionSequenceExecutor _motionSequenceExecutor;
};

#endif /* UR_CONTROL_COMMON_MOTIONSEXECUTOR_H_ */
