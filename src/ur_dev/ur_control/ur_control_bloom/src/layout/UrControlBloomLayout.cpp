//Corresponding header
#include "ur_control_bloom/layout/UrControlBloomLayout.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/layout/config/UrControlBloomLayoutConfig.h"
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

namespace {
const std::string STATE_MACHINE_VISUALS_TEXT_PREFIX = "State Name:   ";
}

ErrorCode UrControlBloomLayout::init(
  const UrControlBloomLayoutConfig& cfg,
  const UrControlCommonLayoutOutInterface& commonOutInterface,
  UrControlCommonLayoutInterface &commonInterface) {

  if (ErrorCode::SUCCESS != UrControlCommonLayout::init(cfg.commonLayoutCfg,
          commonOutInterface, commonInterface)) {
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
  _stateText.setText(
    (STATE_MACHINE_VISUALS_TEXT_PREFIX + BloomState::INIT).c_str());
}

void UrControlBloomLayout::exitInitState() {

}

void UrControlBloomLayout::enterIdleState() {
  _stateText.setText(
    (STATE_MACHINE_VISUALS_TEXT_PREFIX + BloomState::IDLE).c_str());
}

void UrControlBloomLayout::exitIdleState() {

}

void UrControlBloomLayout::enterBloomState() {
  _rose.show();
  _stateText.setText(
    (STATE_MACHINE_VISUALS_TEXT_PREFIX + BloomState::BLOOM).c_str());
}

void UrControlBloomLayout::exitBloomState() {
  _rose.hide();
}

void UrControlBloomLayout::enterBloomRecoveryState() {
  _stateText.setText(
    (STATE_MACHINE_VISUALS_TEXT_PREFIX + BloomState::BLOOM_RECOVERY).c_str());
}

void UrControlBloomLayout::exitBloomRecoveryState() {

}

void UrControlBloomLayout::enterJengaState() {
  _jenga.show();
  _stateText.setText(
    (STATE_MACHINE_VISUALS_TEXT_PREFIX + BloomState::JENGA).c_str());
}

void UrControlBloomLayout::exitJengaState() {
  _jenga.hide();
}

void UrControlBloomLayout::enterJengaRecoveryState() {
  _stateText.setText(
    (STATE_MACHINE_VISUALS_TEXT_PREFIX + BloomState::JENGA_RECOVERY).c_str());
}

void UrControlBloomLayout::exitJengaRecoveryState() {

}

ErrorCode UrControlBloomLayout::initStandaloneEntities(
  const UrControlBloomLayoutConfig &cfg) {
  _rose.create(cfg.roseRsrcId);
  _jenga.create(cfg.jengaRsrcId);

  _stateText.create(cfg.stateVisualsFontRsrcId, 
    STATE_MACHINE_VISUALS_TEXT_PREFIX.c_str(), Colors::BLACK);

  //get position of text positioned below
  Point stateTextPos = safetyModeVisuals.getUpperLeftBoundaryPos();

  //translate the text above
  stateTextPos.y -= STATUS_VISUALS_TEXTS_Y_OFFSET;
  _stateText.setPosition(stateTextPos);

  return ErrorCode::SUCCESS;                                    
}