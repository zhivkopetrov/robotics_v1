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

  if (TouchEvent::KEYBOARD_RELEASE == e.type) {
    if (Keyboard::KEY_T == e.key) {
      _stateMachine.changeState(BloomState::BLOOM_RECOVERY);
    }
    else if (Keyboard::KEY_Y == e.key) {
      _stateMachine.changeState(BloomState::IDLE);
    }
    else if (Keyboard::KEY_U == e.key) {
      _stateMachine.changeState(BloomState::BLOOM);
    }
    else if (Keyboard::KEY_I == e.key) {
      _stateMachine.changeState(BloomState::JENGA);
    }
    else if (Keyboard::KEY_ENTER == e.key) {
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
}

void UrControlBloom::exitInitState() {
  _layout.exitInitState();
}

void UrControlBloom::enterIdleState() {
  _layout.enterIdleState();
}

void UrControlBloom::exitIdleState() {
  _layout.exitIdleState();
}

void UrControlBloom::enterBloomState() {
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
  _layout.enterBloomRecoveryState();
}

void UrControlBloom::exitBloomRecoveryState() {
  _layout.exitBloomRecoveryState();
}

void UrControlBloom::enterJengaState() {
  _layout.enterJengaState();
  _motionExecutor.loadSequence(Motion::JENGA_MOTION_ID);

  const auto doneCb = [this](){
    _stateMachine.changeState(BloomState::BLOOM);
  };
  _motionExecutor.performAction(MotionAction::START, doneCb);
}

void UrControlBloom::exitJengaState() {
  _layout.exitJengaState();
}

void UrControlBloom::enterJengaRecoveryState() {
  _layout.enterJengaRecoveryState();
}

void UrControlBloom::exitJengaRecoveryState() {
  _layout.exitJengaRecoveryState();
}

