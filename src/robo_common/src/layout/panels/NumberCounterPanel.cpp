//Corresponding header
#include "robo_common/layout/panels/NumberCounterPanel.h"

//System headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

ErrorCode NumberCounterPanel::init(
    const NumberCounterPanelConfig& cfg,
    const NumberCounterPanelUtilityConfig &utilityCfg) {
  if (nullptr == utilityCfg.targetReachedCb) {
    LOGERR("Error, nullptr provided for NumberCounterTargetReachedCb");
    return ErrorCode::FAILURE;
  }
  _targetReachedCb = utilityCfg.targetReachedCb;

  NumberCounterConfig numberPanelCfg;
  numberPanelCfg.backgroundRsrcId = cfg.rsrcId;
  numberPanelCfg.backgroundRsrcPos = Point(utilityCfg.pos);
  numberPanelCfg.fontId = cfg.fontId;
  numberPanelCfg.fontColor = utilityCfg.textColor;
  numberPanelCfg.incrTimerId = cfg.incrTimerId;
  numberPanelCfg.decrTimerId = cfg.decrTimerId;
  numberPanelCfg.startValue = cfg.startValue;
  numberPanelCfg.boundaryRect =
      Rectangle(utilityCfg.pos.x + 50, utilityCfg.pos.y + 15, 346, 120);

  auto& triggerCfg = numberPanelCfg.triggerCfg;
  triggerCfg.isIncreasingTrigger = true;
  triggerCfg.value = cfg.targetNumber;
  triggerCfg.triggerCb = std::bind(
      &NumberCounterPanel::onTargetCounterReached, this, std::placeholders::_1);

  if (ErrorCode::SUCCESS != _numberPanel.init(numberPanelCfg)) {
    LOGERR("Error, _numberPanel.init() failed");
    return ErrorCode::FAILURE;
  }

  std::string textContent = "/ ";
  textContent.append(std::to_string(cfg.targetNumber));
  _totalCoinsText.create(cfg.fontId, textContent.c_str(), utilityCfg.textColor,
      Point(utilityCfg.pos.x + 290, utilityCfg.pos.y + 30));

  return ErrorCode::SUCCESS;
}

void NumberCounterPanel::draw() const {
  _numberPanel.draw();
  _totalCoinsText.draw();
}

void NumberCounterPanel::increaseCounter(int32_t delta) {
  _numberPanel.increaseWith(delta);
}

void NumberCounterPanel::onTargetCounterReached(
    [[maybe_unused]]uint64_t target) {
  _targetReachedCb();
}

