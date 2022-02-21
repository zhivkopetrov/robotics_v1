//Corresponding header
#include "robo_collector_gui/layout/panels/CoinPanel.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t CoinPanel::init(const CoinPanelConfig& cfg,
                        const GameWonCb &gameWonCb) {
  if (nullptr == gameWonCb) {
    LOGERR("Error, nullptr provided for GameWonCb");
    return FAILURE;
  }
  _gameWonCb = gameWonCb;

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
  triggerCfg.value = cfg.targetCoins;
  triggerCfg.triggerCb =
      std::bind(&CoinPanel::onTargetCoinsReached, this, std::placeholders::_1);

  if (SUCCESS != _numberPanel.init(numberPanelCfg)) {
    LOGERR("Error, _numberPanel.init() failed");
    return FAILURE;
  }

  std::string textContent = "/ ";
  textContent.append(std::to_string(cfg.targetCoins));
  _totalCoinsText.create(cfg.fontId, textContent.c_str(), lightGoldColor,
      Point(panelX + 290, panelY + 30));

  return SUCCESS;
}

void CoinPanel::draw() const {
  _numberPanel.draw();
  _totalCoinsText.draw();
}

void CoinPanel::increaseCollectedCoins(int32_t coins) {
  _numberPanel.increaseWith(coins);
}

void CoinPanel::onTargetCoinsReached([[maybe_unused]]uint64_t targetCoins) {
  _gameWonCb();
}

