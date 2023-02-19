//Corresponding header
#include "ur_control_bloom/UrControlBloom.h"

//System headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/defines/UrControlBloomDefines.h"
#include "ur_control_bloom/helpers/UrControlBloomInitHelper.h"

UrControlBloom::UrControlBloom(
    const Ros2CommunicatorInterface &communicatorOutInterface)
    : _communicatorInterface(communicatorOutInterface) {

}

ErrorCode UrControlBloom::init(const std::any &cfg) {
  if (ErrorCode::SUCCESS != UrControlBloomInitHelper::init(cfg, *this)) {
    LOGERR("Error, UrControlBloomInitHelper::init() failed");
    return ErrorCode::FAILURE;
  }

  _communicatorInterface.registerNodeCb(_dashboardProvider);
  _communicatorInterface.registerNodeCb(_externalBridge);
  return ErrorCode::SUCCESS;
}

void UrControlBloom::deinit() {
  _communicatorInterface.unregisterNodeCb(_dashboardProvider);
  _communicatorInterface.unregisterNodeCb(_externalBridge);
  _dashboardProvider->deinit();
  _motionExecutor.shutdown();
  _stateMachine.shutdown();
  _layout.deinit();
}

void UrControlBloom::draw() const {
  _layout.draw();
}

void UrControlBloom::handleEvent(const InputEvent &e) {
  _layout.handleEvent(e);

  //TODO extend the state machine to accept inputs events
  //pass the input even to the active state
  if (TouchEvent::KEYBOARD_RELEASE == e.type) {
    if (Keyboard::KEY_U == e.key) {
      _stateMachine.changeState(BloomState::BLOOM);
    }
    else if (Keyboard::KEY_I == e.key) {
      _stateMachine.changeState(BloomState::JENGA);
    }
    else if ((Keyboard::KEY_ENTER == e.key) || 
             (Keyboard::KEY_NUMPAD_ENTER == e.key)) {
      const auto doneCb = [this](){
        _stateMachine.changeState(BloomState::BLOOM);
      };

      _motionExecutor.loadSequence(Motion::JENGA_MOTION_ID);
      _motionExecutor.performAction(MotionAction::GRACEFUL_STOP, doneCb);
    }
    else if (Keyboard::KEY_BACKSPACE == e.key) {
      const auto doneCb = [this](){
        _stateMachine.changeState(BloomState::IDLE);
      };

      _motionExecutor.performAction(MotionAction::ABORT, doneCb);
    }
  }
}

void UrControlBloom::process() {

}

void UrControlBloom::enterInitState() {
  _layout.enterInitState();

  const auto f = [this]() {
    //TODO initialise and break release, 
    //if robot mode or safety mode are not proper

    //TODO2 initialise Tool Center Point (TCP) and Center of Gravity (CoG)

    return [this, transitionState = getRecoveryTransitionStateName()](){
      _stateMachine.changeState(transitionState);
    }();
  };

  _invokeActionEventCb(f, ActionEventType::NON_BLOCKING);
}

void UrControlBloom::exitInitState() {
  _layout.exitInitState();
}

void UrControlBloom::enterIdleState() {
  serializeState(BloomState::IDLE);
  _layout.enterIdleState();
}

void UrControlBloom::exitIdleState() {
  _layout.exitIdleState();
}

void UrControlBloom::enterBloomState() {
  serializeState(BloomState::BLOOM);
  _layout.enterBloomState();

  _motionExecutor.loadSequence(Motion::BLOOM_MOTION_ID);
  const auto doneCb = [this](){
    _stateMachine.changeState(BloomState::JENGA);
  };
  _motionExecutor.performAction(MotionAction::START, doneCb);
}

void UrControlBloom::exitBloomState() {
  _layout.exitBloomState();
}

void UrControlBloom::enterBloomRecoveryState() {
  serializeState(BloomState::BLOOM_RECOVERY);
  _layout.enterBloomRecoveryState();

  _motionExecutor.loadSequence(Motion::BLOOM_MOTION_ID);
  const auto doneCb = [this](){
    _stateMachine.changeState(BloomState::BLOOM);
  };
  _motionExecutor.performAction(MotionAction::RECOVER, doneCb);
}

void UrControlBloom::exitBloomRecoveryState() {
  _layout.exitBloomRecoveryState();
}

void UrControlBloom::enterJengaState() {
  serializeState(BloomState::JENGA);
  _layout.enterJengaState();

  _motionExecutor.loadSequence(Motion::JENGA_MOTION_ID);
  const auto doneCb = [this](){
    _stateMachine.changeState(BloomState::IDLE);
  };
  _motionExecutor.performAction(MotionAction::START, doneCb);
}

void UrControlBloom::exitJengaState() {
  _layout.exitJengaState();
}

void UrControlBloom::enterJengaRecoveryState() {
  serializeState(BloomState::JENGA_RECOVERY);
  _layout.enterJengaRecoveryState();

  _motionExecutor.loadSequence(Motion::JENGA_MOTION_ID);
  const auto doneCb = [this](){
    _stateMachine.changeState(BloomState::JENGA);
  };
  _motionExecutor.performAction(MotionAction::RECOVER, doneCb);
}

void UrControlBloom::exitJengaRecoveryState() {
  _layout.exitJengaRecoveryState();
}

std::string UrControlBloom::getRecoveryTransitionStateName() const {
  std::string stateStr;
  const ErrorCode errCode = _stateFileHandler->getEntry(
    BloomState::SECTION_NAME, BloomState::STATE_ENTRY_NAME, 
    stateStr);
  if (ErrorCode::SUCCESS != errCode) {
    LOGERR("Error trying to getEntry(): [%s] for section: [%s]. "
            "Defaulting to [%s]", BloomState::STATE_ENTRY_NAME, 
            BloomState::SECTION_NAME, BloomState::IDLE);
    stateStr = BloomState::IDLE;
  }

  if ((BloomState::BLOOM == stateStr) || 
      (BloomState::BLOOM_RECOVERY == stateStr)) {
    return BloomState::BLOOM_RECOVERY;
  }

  if ((BloomState::JENGA == stateStr) || 
      (BloomState::JENGA_RECOVERY == stateStr)) {
    return BloomState::JENGA_RECOVERY;
  }

  return BloomState::IDLE;
}

void UrControlBloom::serializeState(const std::string& stateName) {
  const ErrorCode errCode = _stateFileHandler->updateEntry(
    BloomState::SECTION_NAME, BloomState::STATE_ENTRY_NAME, stateName);
  if (ErrorCode::SUCCESS != errCode) {
    LOGERR("Error trying to serialize BloomState: [%s]", stateName.c_str());
  }
}
