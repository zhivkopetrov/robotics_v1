//Corresponding header
#include "ur_control_common/motion/MotionSequenceExecutor.h"

//System headers
#include <future>

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
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

  if (_outInterface.invokeURScriptPreemptServiceCb == nullptr) {
    LOGERR("Error, nullptr provided for InvokeURScriptPreemptServiceCb");
    return ErrorCode::FAILURE;
  }

  _commandConsumerThread = 
    std::thread(&MotionSequenceExecutor::runCommandComsumerLoop, this);
  _blockingInvokerThread = 
    std::thread(&MotionSequenceExecutor::runBlockingInvokerLoop, this);

  //detach the invoker thread, because we might want to terminate the program
  //and now wait for it's blocking task to complete
  _blockingInvokerThread.detach();

  return ErrorCode::SUCCESS;
}

void MotionSequenceExecutor::shutdown() {
  _commandQueue.shutdown();
  _blockingTasksQueue.shutdown();
  _commandConsumerThread.join();
}

void MotionSequenceExecutor::dispatchAsync(
  const std::vector<MotionCommand>& commands, 
  const MotionActionDoneCb& actionDoneCb) {
  //_actionDoneMutex and _commandQueue must be in sync
  //otherwise a data race (not data corruption) might occur
  std::lock_guard<std::mutex> lock(_actionDoneMutex);

  if (!_commandQueue.isEmpty()) {
    LOGY("MotionSequenceExecutor: overriding active motion commands");
    _commandQueue.clear();
    _preemptCurrCommand = true;
  }

  _actionDoneCb = actionDoneCb;

  //after the batch is inserted in the queue, a notification signal will
  //wake the internal _thread waiting on the conditional variable
  _commandQueue.pushBatch(commands);
}

void MotionSequenceExecutor::runCommandComsumerLoop() {
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

    invokeCommand(command);

    //_actionDoneMutex and _commandQueue must be in sync
    //otherwise a data race (not data corruption) might occur
    std::lock_guard<std::mutex> lock(_actionDoneMutex);
    if (_commandQueue.isEmpty()) {
      //last command was popped -> invoke the done callback
      _actionDoneCb();
    }
  }
}

void MotionSequenceExecutor::runBlockingInvokerLoop() {
  BlockingTask blockingTask;

  while (true) {
    const auto [isShutdowned, hasTimedOut] = 
      _blockingTasksQueue.waitAndPop(blockingTask);
    if (isShutdowned) {
      break; //stop thread
    }
    if (hasTimedOut) {
      continue;
    }

    blockingTask();
  }
}

void MotionSequenceExecutor::invokeCommand(const MotionCommand& cmd) {
  switch (cmd.policy)
  {
  case MotionExecutionPolicy::NON_BLOCKING:
    _outInterface.publishURScriptCb(cmd.data);
    break;

  case MotionExecutionPolicy::BLOCKING:
    invokeBlockingCommand(cmd.data);
    break;
  
  default:
    LOGERR("Received unsupported MotionExecutionPolicy value: [%d]. Discarding "
           "command: [%s]", getEnumValue(cmd.policy), cmd.data.c_str());
    break;
  }
}

void MotionSequenceExecutor::invokeBlockingCommand(
    const UrScriptPayload& data) {
  BlockingTask blockingTask = [this, &data](){
    _outInterface.invokeURScriptServiceCb(data);
  };

  //NOTE: keep task alive, because otherwise it could lead the
  //      invoking thread with dangling reference to local object
  auto task = std::make_shared<std::packaged_task<void()>>(blockingTask);
  auto future = task->get_future();
  auto f = [task](){
    (*task)();
  };
  _blockingTasksQueue.push(std::move(f));

  //wait for future completion while keeping resposivness
  while (true) {
    if (_preemptCurrCommand) {
      _preemptCurrCommand = false;
      _outInterface.invokeURScriptPreemptServiceCb();
      break;
    }

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
