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
}

void UrControlBloomLayout::exitInitState() {

}

void UrControlBloomLayout::enterIdleState() {
  _stateText.setTextAndColor(BloomState::IDLE, Colors::GREEN);
}

void UrControlBloomLayout::exitIdleState() {

}

void UrControlBloomLayout::enterBloomState() {
  _rose.show();
  _stateText.setTextAndColor(BloomState::BLOOM, Colors::GREEN);
}

void UrControlBloomLayout::exitBloomState() {
  _rose.hide();
}

void UrControlBloomLayout::enterBloomRecoveryState() {
  _stateText.setTextAndColor(BloomState::BLOOM_RECOVERY, Colors::ORANGE);
}

void UrControlBloomLayout::exitBloomRecoveryState() {

}

void UrControlBloomLayout::enterJengaState() {
  _jenga.show();
  _stateText.setTextAndColor(BloomState::JENGA, Colors::GREEN);
}

void UrControlBloomLayout::exitJengaState() {
  _jenga.hide();
}

void UrControlBloomLayout::enterJengaRecoveryState() {
  _stateText.setTextAndColor(BloomState::JENGA_RECOVERY, Colors::ORANGE);
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