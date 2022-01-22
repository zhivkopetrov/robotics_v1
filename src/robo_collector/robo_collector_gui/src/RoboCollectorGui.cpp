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
  using namespace std::placeholders;

  try {
    const auto &gameCfg = std::any_cast<const RoboCollectorGuiConfig&>(cfg);
    if (SUCCESS != _field.init(gameCfg.fieldCfg)) {
      LOGERR("Error in _field.init()");
      return FAILURE;
    }

    _map.create(gameCfg.mapRsrcId);
    const auto setFieldDataMarkerCb =
        std::bind(&Field::setFieldDataMarker, &_field, _1, _2);
    const auto resetFieldDataMarkerCb =
        std::bind(&Field::resetFieldDataMarker, &_field, _1);
    const auto getFieldDataCb = std::bind(&Field::getFieldData, &_field);

    RobotCfg robotCfg;
    robotCfg.collisionCb =
        std::bind(&Panel::decreaseHealthIndicator, &_panel, _1);
    robotCfg.rsrcId = gameCfg.robotBlinkyRsrcId;
    robotCfg.fieldPos.row = 1;
    robotCfg.fieldPos.col = 4;
    robotCfg.frameId = 0;
    robotCfg.animTimerId = gameCfg.robotsAnimStartTimerId;
    robotCfg.setFieldDataMarkerCb = setFieldDataMarkerCb;
    robotCfg.resetFieldDataMarkerCb = resetFieldDataMarkerCb;

    robotCfg.getFieldDataCb = getFieldDataCb;

    robotCfg.fieldMarker = gameCfg.blinkyFieldMarker;
    robotCfg.enemyFieldMarker = gameCfg.enemyFieldMarker;
    if (SUCCESS != _blinky.init(robotCfg)) {
      LOGERR("Error in _field.init()");
      return FAILURE;
    }

    robotCfg.rsrcId = gameCfg.robotEnemiesRsrcId;
    robotCfg.fieldPos.row = 0;
    robotCfg.frameId = 0;
    //reversed for the enemies
    robotCfg.fieldMarker = gameCfg.enemyFieldMarker;
    robotCfg.enemyFieldMarker = gameCfg.blinkyFieldMarker;
    for (auto i = 0; i < Defines::ENEMIES_CTN; ++i) {
      robotCfg.fieldPos.col = i;
      robotCfg.animTimerId = gameCfg.robotsAnimStartTimerId + 1;
      if (SUCCESS != _enemies[i].init(robotCfg)) {
        LOGERR("Error in _field.init()");
        return FAILURE;
      }
      ++robotCfg.frameId;
    }

    if (SUCCESS != _panel.init(gameCfg.panelConfig)) {
      LOGERR("Error in _field.init()");
      return FAILURE;
    }

    CoinHandlerConfig coinHandlerCfg;
    coinHandlerCfg.animRsrcIds = gameCfg.coinAnimRsrcIds;
    coinHandlerCfg.maxCoins = gameCfg.maxCoins;
    coinHandlerCfg.animFirstTimerId = gameCfg.coinAnimFirstTimerId;
    coinHandlerCfg.setFieldDataMarkerCb = setFieldDataMarkerCb;
    coinHandlerCfg.resetFieldDataMarkerCb = resetFieldDataMarkerCb;
    coinHandlerCfg.getFieldDataCb = getFieldDataCb;
    if (SUCCESS != _coinHandler.init(coinHandlerCfg)) {
      LOGERR("Error in _coinHandler.init()");
      return FAILURE;
    }

    MoveButtonCfg moveButtonCfg;
    const std::array<Point, MOVE_BUTTONS_CTN> buttonsPos {
      Point(1435, 695), Point(1285, 860), Point(1585, 860)
    };
    const std::array<MoveType, MOVE_BUTTONS_CTN> buttonsMoveType {
      MoveType::FORWARD, MoveType::ROTATE_LEFT, MoveType::ROTATE_RIGHT
    };
    const std::array<uint64_t, MOVE_BUTTONS_CTN> buttonsRsrcIds {
      gameCfg.upMoveButtonRsrcId, gameCfg.leftMoveButtonRsrcId,
      gameCfg.rightMoveButtonRsrcId
    };
    moveButtonCfg.moveCb = std::bind(&Robot::act, &_blinky, _1);

    for (auto i = 0; i < MOVE_BUTTONS_CTN; ++i) {
      moveButtonCfg.startPos = buttonsPos[i];
      moveButtonCfg.moveType = buttonsMoveType[i];
      moveButtonCfg.rsrcId = buttonsRsrcIds[i];
      if (SUCCESS != _moveButtons[i].init(moveButtonCfg)) {
        LOGERR("Error in _moveButtons[%d].init()", i);
        return FAILURE;
      }
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
  _map.draw();
  _field.draw();
  _panel.draw();
  _coinHandler.draw();
  _blinky.draw();

  for (const auto &enemy : _enemies) {
    enemy.draw();
  }
  for (const auto &button : _moveButtons) {
    button.draw();
  }
}

void RoboCollectorGui::handleEvent(const InputEvent &e) {
  for (auto& button : _moveButtons) {
    if(button.isInputUnlocked() && button.containsEvent(e)) {
      button.handleEvent(e);
      break;
    }
  }
}

