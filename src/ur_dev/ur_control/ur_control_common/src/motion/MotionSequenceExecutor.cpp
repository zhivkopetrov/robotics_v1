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

  if (_outInterface.invokeActionEventCb == nullptr) {
    LOGERR("Error, nullptr provided for InvokeActionEventCb");
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

void MotionSequenceExecutor::dispatchUscriptsAsync(
  const std::vector<UscriptCommand>& commands, 
  const UscriptsBatchDoneCb& batchDoneCb) {
  //_uscriptsBatchDoneMutex and _commandQueue must be in sync
  //otherwise a data race (not data corruption) might occur
  std::lock_guard<std::mutex> lock(_uscriptsBatchDoneMutex);

  if (!_commandQueue.isEmpty()) {
    LOGY("MotionSequenceExecutor: overriding active motion commands");
    _commandQueue.clear();
    _preemptCurrCommand = true;
  }

  _uscriptsBatchDoneCb = batchDoneCb;

  //after the batch is inserted in the queue, a notification signal will
  //wake the internal _thread waiting on the conditional variable
  _commandQueue.pushBatch(commands);
}

void MotionSequenceExecutor::runCommandComsumerLoop() {
  UscriptCommand command;

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

    //_uscriptsBatchDoneMutex and _commandQueue must be in sync
    //otherwise a data race (not data corruption) might occur
    std::lock_guard<std::mutex> lock(_uscriptsBatchDoneMutex);
    if (_commandQueue.isEmpty()) {
      //last command was popped -> invoke the done callback
      //through the ActionEventSystem
      //make a copy, because the lambda can be changed
      const auto uscriptsBatchDoneCbCopy = _uscriptsBatchDoneCb;
      const auto f = [uscriptsBatchDoneCbCopy](){
        uscriptsBatchDoneCbCopy();
      };
      _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
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

void MotionSequenceExecutor::invokeCommand(const UscriptCommand& cmd) {
  switch (cmd.policy)
  {
  case MotionExecutionPolicy::NON_BLOCKING:
    invokeNonBlockingCommand(cmd);
    break;

  case MotionExecutionPolicy::BLOCKING:
    invokeBlockingCommand(cmd);
    break;
  
  default:
    LOGERR("Received unsupported MotionExecutionPolicy value: [%d]. Discarding "
           "command: [%s]", getEnumValue(cmd.policy), cmd.data.c_str());
    break;
  }
}

void MotionSequenceExecutor::invokeNonBlockingCommand(
  const UscriptCommand& cmd) {
  _outInterface.publishURScriptCb(cmd.data);
  if (cmd.doneCb) {
    const auto doneCbCopy = cmd.doneCb;
    const auto f = [doneCbCopy](){
      doneCbCopy();
    };
    _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
  }
}

void MotionSequenceExecutor::invokeBlockingCommand(const UscriptCommand& cmd) {
  const BlockingTask blockingTask = [this, &cmd](){
    _outInterface.invokeURScriptServiceCb(cmd.data);
  };

  //NOTE: keep task alive, because otherwise it could lead the
  //      invoking thread with dangling reference to local object
  auto packagedTask =
    std::make_shared<std::packaged_task<void()>>(blockingTask);
  auto future = packagedTask->get_future();
  auto task = [packagedTask](){
    (*packagedTask)();
  };
  _blockingTasksQueue.push(std::move(task));

  //wait for future completion while keeping responsiveness
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
      if (cmd.doneCb) {
        const auto doneCbCopy = cmd.doneCb;
        const auto f = [doneCbCopy](){
          doneCbCopy();
        };
        _outInterface.invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
      }

      break;
    }
  }
}
