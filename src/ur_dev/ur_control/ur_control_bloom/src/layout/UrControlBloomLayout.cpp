//Corresponding header
#include "ur_control_bloom/layout/UrControlBloomLayout.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/layout/config/UrControlBloomLayoutConfig.h"
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

ErrorCode UrControlBloomLayout::init(
  const UrControlBloomLayoutConfig& cfg,
  const UrControlBloomLayoutOutInterface& outInterface,
  UrControlBloomLayoutInterface& interface) {

  if (ErrorCode::SUCCESS != UrControlCommonLayout::init(cfg.commonLayoutCfg,
          outInterface.commonOutInterface, interface.commonInterface)) {
    LOGERR("UrControlCommonLayout::init() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initStandaloneEntities(cfg)) {
    LOGERR("initStandaloneImages() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void UrControlBloomLayout::deinit() {
  UrControlCommonLayout::deinit();
}

void UrControlBloomLayout::draw() const {
  UrControlCommonLayout::draw();
  _rose.draw();
  _jenga.draw();
  _stateTextHeader.draw();
  _stateText.draw();
}

void UrControlBloomLayout::handleEvent(const InputEvent &e) {
  UrControlCommonLayout::handleEvent(e);
}

void UrControlBloomLayout::process() {

}

void UrControlBloomLayout::enterInitState() {
  _jenga.hide();
  _rose.hide();
  _stateText.setTextAndColor(BloomState::INIT, Colors::BLACK);

  const std::vector<int32_t> lockBtnIndexes = {};
  const std::vector<int32_t> unlockBtnIndexes = { 
    JENGA_IDX, BLOOM_RANDOMIZED_IDX, BLOOM_1ST_IDX, BLOOM_2ND_IDX, 
    BLOOM_3RD_IDX, ABORT_MOTION_IDX, PARK_IDX
  };
  if (ErrorCode::SUCCESS != buttonHandler->setCommandButtonsLockStatus(
    lockBtnIndexes, unlockBtnIndexes)) {
    LOGERR("Error, setCommandButtonsLockStatus() failed");
  }
  buttonHandler->setGripperButtonsLockStatus(
    GripperButtonsInputStatus::UNLOCKED);
}

void UrControlBloomLayout::exitInitState() {

}

void UrControlBloomLayout::enterIdleState() {
  _stateText.setTextAndColor(BloomState::IDLE, Colors::GREEN);

  const std::vector<int32_t> lockBtnIndexes = { ABORT_MOTION_IDX };
  const std::vector<int32_t> unlockBtnIndexes = { 
    JENGA_IDX, BLOOM_RANDOMIZED_IDX, BLOOM_1ST_IDX, BLOOM_2ND_IDX, 
    BLOOM_3RD_IDX, PARK_IDX
  };
  if (ErrorCode::SUCCESS != buttonHandler->setCommandButtonsLockStatus(
    lockBtnIndexes, unlockBtnIndexes)) {
    LOGERR("Error, setCommandButtonsLockStatus() failed");
  }
  buttonHandler->setGripperButtonsLockStatus(
    GripperButtonsInputStatus::UNLOCKED);
}

void UrControlBloomLayout::exitIdleState() {

}

void UrControlBloomLayout::enterBloomState() {
  _rose.show();
  _stateText.setTextAndColor(BloomState::BLOOM, Colors::GREEN);

  const std::vector<int32_t> lockBtnIndexes = { 
    JENGA_IDX, BLOOM_RANDOMIZED_IDX, BLOOM_1ST_IDX, BLOOM_2ND_IDX, 
    BLOOM_3RD_IDX, PARK_IDX
  };
  const std::vector<int32_t> unlockBtnIndexes = { ABORT_MOTION_IDX };
  if (ErrorCode::SUCCESS != buttonHandler->setCommandButtonsLockStatus(
    lockBtnIndexes, unlockBtnIndexes)) {
    LOGERR("Error, setCommandButtonsLockStatus() failed");
  }
  buttonHandler->setGripperButtonsLockStatus(GripperButtonsInputStatus::LOCKED);
}

void UrControlBloomLayout::exitBloomState() {
  _rose.hide();
}

void UrControlBloomLayout::enterBloomRecoveryState() {
  _stateText.setTextAndColor(BloomState::BLOOM_RECOVERY, Colors::ORANGE);

  const std::vector<int32_t> lockBtnIndexes = { 
    JENGA_IDX, BLOOM_RANDOMIZED_IDX, BLOOM_1ST_IDX, BLOOM_2ND_IDX, 
    BLOOM_3RD_IDX, PARK_IDX
  };
  const std::vector<int32_t> unlockBtnIndexes = { ABORT_MOTION_IDX };
  if (ErrorCode::SUCCESS != buttonHandler->setCommandButtonsLockStatus(
    lockBtnIndexes, unlockBtnIndexes)) {
    LOGERR("Error, setCommandButtonsLockStatus() failed");
  }
  buttonHandler->setGripperButtonsLockStatus(GripperButtonsInputStatus::LOCKED);
}

void UrControlBloomLayout::exitBloomRecoveryState() {

}

void UrControlBloomLayout::enterJengaState() {
  _jenga.show();
  _stateText.setTextAndColor(BloomState::JENGA, Colors::GREEN);

  const std::vector<int32_t> lockBtnIndexes = { 
    JENGA_IDX, BLOOM_RANDOMIZED_IDX, BLOOM_1ST_IDX, BLOOM_2ND_IDX, 
    BLOOM_3RD_IDX, PARK_IDX
  };
  const std::vector<int32_t> unlockBtnIndexes = { ABORT_MOTION_IDX };
  if (ErrorCode::SUCCESS != buttonHandler->setCommandButtonsLockStatus(
    lockBtnIndexes, unlockBtnIndexes)) {
    LOGERR("Error, setCommandButtonsLockStatus() failed");
  }
  buttonHandler->setGripperButtonsLockStatus(GripperButtonsInputStatus::LOCKED);
}

void UrControlBloomLayout::exitJengaState() {
  _jenga.hide();
}

void UrControlBloomLayout::enterJengaRecoveryState() {
  _stateText.setTextAndColor(BloomState::JENGA_RECOVERY, Colors::ORANGE);

  const std::vector<int32_t> lockBtnIndexes = { 
    JENGA_IDX, BLOOM_RANDOMIZED_IDX, BLOOM_1ST_IDX, BLOOM_2ND_IDX, 
    BLOOM_3RD_IDX, PARK_IDX
  };
  const std::vector<int32_t> unlockBtnIndexes = { ABORT_MOTION_IDX };
  if (ErrorCode::SUCCESS != buttonHandler->setCommandButtonsLockStatus(
    lockBtnIndexes, unlockBtnIndexes)) {
    LOGERR("Error, setCommandButtonsLockStatus() failed");
  }
  buttonHandler->setGripperButtonsLockStatus(GripperButtonsInputStatus::LOCKED);
}

void UrControlBloomLayout::exitJengaRecoveryState() {

}

ErrorCode UrControlBloomLayout::initStandaloneEntities(
  const UrControlBloomLayoutConfig &cfg) {
  _rose.create(cfg.roseRsrcId);
  _jenga.create(cfg.jengaRsrcId);

  _stateTextHeader.create(
    cfg.stateVisualsFontRsrcId, "State Name:   ", Colors::BLACK);

  //get position of text positioned below
  Point stateTextHeaderPos = safetyModeVisuals.getUpperLeftBoundaryPos();

  //translate the text above
  stateTextHeaderPos.y -= STATUS_VISUALS_TEXTS_Y_OFFSET;
  _stateTextHeader.setPosition(stateTextHeaderPos);

  const Point stateTextPos = 
  Point(stateTextHeaderPos.x + _stateTextHeader.getImageWidth(), 
        stateTextHeaderPos.y);
  _stateText.create(
    cfg.stateVisualsFontRsrcId, " ", Colors::BLACK, stateTextPos);

  return ErrorCode::SUCCESS;                                    
}