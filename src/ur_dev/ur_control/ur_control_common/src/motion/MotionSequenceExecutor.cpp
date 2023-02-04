//Corresponding header
#include "ur_control_common/motion/MotionSequenceExecutor.h"

//System headers
#include <future>

//Other libraries headers
#include "utils/Log.h"

//Own components headers

ErrorCode MotionSequenceExecutor::init(
  const MotionSequenceExecutorOutInterface& outInterface) {
  _outInterface = outInterface;
  if (_outInterface.publishURScriptCb == nullptr) {
    LOGERR("Error, nullptr provided for PublishURScriptCb");
    return ErrorCode::FAILURE;
  }

  if (_outInterface.invokeURScriptServiceCb == nullptr) {
    LOGERR("Error, nullptr provided for InvokeURScriptServiceCb");
    return ErrorCode::FAILURE;
  }

  _thread = std::thread(&MotionSequenceExecutor::runInternalThread, this);

  return ErrorCode::SUCCESS;
}

void MotionSequenceExecutor::shutdown() {
  _commandQueue.shutdown();
  _thread.join();
}

void MotionSequenceExecutor::dispatchAsync(
  const std::vector<MotionCommand>& commands, 
  const MotionActionDoneCb& actionDoneCb) {
  //_actionDoneMutex and _commandQueue must be in sync
  //otherwise a data race (not data corruption) might occur
  std::lock_guard<std::mutex> lock(_actionDoneMutex);

  if (!_commandQueue.isEmpty()) {
    LOGR("MotionSequenceExecutor: overriding active motion commands");
    _commandQueue.clear();
  }

  _actionDoneCb = actionDoneCb;

  //after the batch is inserted in the queue, a notification signal will
  //wake the internal _thread waiting on the conditional variable
  _commandQueue.pushBatch(commands);
}

void MotionSequenceExecutor::runInternalThread() {
  MotionCommand command;

  while (true) {
    const auto [isShutdowned, hasTimedOut] = 
      _commandQueue.waitAndPop(command);
    if (isShutdowned) {
      break; //stop thread
    }
    if (hasTimedOut) {
      continue;
    }

    invokeSingleCommand(command);

    //_actionDoneMutex and _commandQueue must be in sync
    //otherwise a data race (not data corruption) might occur
    std::lock_guard<std::mutex> lock(_actionDoneMutex);
    if (_commandQueue.isEmpty()) {
      //last command was popped -> invoke the done callback
      _actionDoneCb();
    }
  }
}

void MotionSequenceExecutor::invokeSingleCommand(const MotionCommand& cmd) {
  if (MotionExecutionPolicy::NON_BLOCKING == cmd.policy) {
    _outInterface.publishURScriptCb(cmd.data);
    return;
  }
 
  //MotionExecutionPolicy::BLOCKING
  const auto f = [this, &cmd](){
    _outInterface.invokeURScriptServiceCb(cmd.data);
  };

  //use std::packaged_task instead of std::async, because 
  //std::async will block waiting on its destructor
  std::packaged_task task(f);
  std::future future = task.get_future();

  //wait for future completion while keeping resposivness
  while (true) {
    if (_commandQueue.isShutDowned()) {
      break;
    }

    using namespace std::literals;
    const auto status = future.wait_for(10ms);
    if (std::future_status::ready == status) {
      break;
    }
  }
}