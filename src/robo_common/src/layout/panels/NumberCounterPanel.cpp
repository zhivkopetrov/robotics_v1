//Corresponding header
#include "robo_common/layout/panels/NumberCounterPanel.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t NumberCounterPanel::init(
    const NumberCounterPanelConfig& cfg,
    const NumberCounterTargetReachedCb &targetReachedCb) {
  if (nullptr == targetReachedCb) {
    LOGERR("Error, nullptr provided for StartGameWonAnimCb");
    return FAILURE;
  }
  _targetReachedCb = targetReachedCb;

  constexpr auto panelX = 1250;
  constexpr auto panelY = 215;
  const auto lightGoldColor = Color(0xD4AF37FF);
  NumberCounterConfig numberPanelCfg;
  numberPanelCfg.backgroundRsrcId = cfg.rsrcId;
  numberPanelCfg.backgroundRsrcPos = Point(panelX, panelY);
  numberPanelCfg.fontId = cfg.fontId;
  numberPanelCfg.fontColor = lightGoldColor;
  numberPanelCfg.incrTimerId = cfg.incrTimerId;
  numberPanelCfg.decrTimerId = cfg.decrTimerId;
  numberPanelCfg.startValue = 0;
  numberPanelCfg.boundaryRect = Rectangle(panelX + 50, panelY + 15, 346, 120);

  auto& triggerCfg = numberPanelCfg.triggerCfg;
  triggerCfg.isIncreasingTrigger = true;
  triggerCfg.value = cfg.targetNumber;
  triggerCfg.triggerCb = std::bind(
      &NumberCounterPanel::onTargetCounterReached, this, std::placeholders::_1);

  if (SUCCESS != _numberPanel.init(numberPanelCfg)) {
    LOGERR("Error, _numberPanel.init() failed");
    return FAILURE;
  }

  std::string textContent = "/ ";
  textContent.append(std::to_string(cfg.targetNumber));
  _totalCoinsText.create(cfg.fontId, textContent.c_str(), lightGoldColor,
      Point(panelX + 290, panelY + 30));

  return SUCCESS;
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

