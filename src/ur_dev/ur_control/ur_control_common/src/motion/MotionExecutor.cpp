//Corresponding header
#include "ur_control_common/motion/MotionExecutor.h"

//System headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

ErrorCode MotionExecutor::init(
  const MotionSequenceExecutorOutInterface& outInterface) {
  if (ErrorCode::SUCCESS != _motionSequenceExecutor.init(outInterface)) {
    LOGERR("Error in _motionSequenceExecutor.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode MotionExecutor::addSequence(
  std::unique_ptr<MotionSequence> sequence) {
  const int32_t id = sequence->getId();
  const auto [it, success] = 
    _supportedSequences.try_emplace(id, std::move(sequence));
  if (!success) {
    LOGERR("Provided MotionSequence: [%s] has an id: [%d], which is already "
           "present in the supported sequences collection. Discarding action", 
           sequence->getName().c_str(), id);
    return ErrorCode::FAILURE;
  }

  using namespace std::placeholders;
  MotionSequence& insertedSequence = *it->second;
  insertedSequence.setDispatchMotionsAsyncCb(std::bind(
    &MotionSequenceExecutor::dispatchAsync, &_motionSequenceExecutor, _1, _2));
  return ErrorCode::SUCCESS;
}

ErrorCode MotionExecutor::loadSequence(int32_t id) {
  auto it = _supportedSequences.find(id);
  if (it == _supportedSequences.end()) {
    LOGERR("Provided sequenceId: [%d] is not a supported sequence id", id);
    return ErrorCode::FAILURE;
  }

  _currSequenceId = id;
  return ErrorCode::SUCCESS;
}

ErrorCode MotionExecutor::performAction(
  MotionAction action, const MotionCommandBatchDoneCb& doneCb) {
  if (EMPTY_SEQUENCE_ID == _currSequenceId) {
    LOGERR("No current MotionSequence loaded");
    return ErrorCode::FAILURE;
  }

  const auto& motionSequence = _supportedSequences[_currSequenceId];
  switch (action) {
  case MotionAction::START:
    motionSequence->start(doneCb);
    break;

  case MotionAction::GRACEFUL_STOP:
    motionSequence->gracefulStop(doneCb);
    break;

  case MotionAction::ABORT:
    motionSequence->abort(doneCb);
    break;

  case MotionAction::RECOVER:
    motionSequence->recover(doneCb);
    break;
  
  default:
    LOGERR(
      "Received unsupported MotionAction type: [%d] for MotionSeuqnce: [%s]",
      getEnumValue(action), motionSequence->getName().c_str());
    ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void MotionExecutor::shutdown() {
  _motionSequenceExecutor.shutdown();
}