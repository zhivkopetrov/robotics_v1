//Corresponding header
#include "robo_collector_gui/RoboCollectorGui.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "sdl_utils/input/InputEvent.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/config/RoboCollectorGuiConfig.h"

int32_t RoboCollectorGui::init(const std::any &cfg) {
  try {
    const auto &gameCfg = std::any_cast<const RoboCollectorGuiConfig&>(cfg);
    if (SUCCESS != _field.init(gameCfg.fieldCfg)) {
      LOGERR("Error in _field.init()");
      return FAILURE;
    }
    constexpr auto tileSize = 160;
    constexpr auto startX = 47;
    constexpr auto startY = 47;

    RobotCfg robotCfg;
    robotCfg.rsrcId = gameCfg.robotEnemiesRsrcId;
    robotCfg.startPos.x = startX;
    robotCfg.startPos.y = startY;
    robotCfg.frameId = 0;
    for (auto &enemy : _enemies) {
      robotCfg.startPos.x += 160;
      if (SUCCESS != enemy.init(robotCfg)) {
        LOGERR("Error in _field.init()");
        return FAILURE;
      }
      ++robotCfg.frameId;
    }

    robotCfg.rsrcId = gameCfg.robotBlinkyRsrcId;
    robotCfg.startPos.x = startX + tileSize;
    robotCfg.startPos.y = startY + tileSize;
    robotCfg.frameId = 0;
    if (SUCCESS != _blinky.init(robotCfg)) {
      LOGERR("Error in _field.init()");
      return FAILURE;
    }

    if (SUCCESS != _panel.init(gameCfg.panelConfig)) {
      LOGERR("Error in _field.init()");
      return FAILURE;
    }

    CoinConfig coinCfg;
    constexpr auto coinOffsetFromTile = 30;
    coinCfg.pos.x = startX + tileSize + coinOffsetFromTile;
    coinCfg.pos.y = startX + coinOffsetFromTile + (3 * tileSize);
    coinCfg.rsrcId = gameCfg.coinAnimRsrcId;
    coinCfg.timerId = gameCfg.cointAnimTimerId;
    if (SUCCESS != _coin.init(coinCfg)) {
      LOGERR("Error in _coin.init()");
      return FAILURE;
    }

  } catch (const std::bad_any_cast &e) {
    LOGERR("std::any_cast<GuiConfig&> failed, %s", e.what());
    return FAILURE;
  }



  return SUCCESS;
}

void RoboCollectorGui::deinit() {

}

void RoboCollectorGui::draw() const {
  _field.draw();
  _panel.draw();
  _coin.draw();

  _blinky.draw();
  for (const auto &enemy : _enemies) {
    enemy.draw();
  }
}

void RoboCollectorGui::handleEvent(const InputEvent &e) {
  _field.handleEvent(e);

  _blinky.handleEvent(e);

  //TODO remove snipper below. used only for test
  if (TouchEvent::KEYBOARD_RELEASE != e.type) {
    return;
  }

  switch (e.key) {
  case Keyboard::KEY_O:
    _panel.shrinkHealthIndicator(5);
    break;

  case Keyboard::KEY_P:
    _panel.shrinkHealthIndicator(-5);
    break;

  default:
    break;
  }
}

