//Corresponding header
#include "ur_control_gui/layout/helpers/UrControlGuiLayoutInitHelper.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/layout/config/UrControlGuiLayoutConfig.h"
#include "ur_control_gui/layout/UrControlGuiLayout.h"

ErrorCode UrControlGuiLayoutInitHelper::init(
    const UrControlGuiLayoutConfig &cfg,
    const UrControlGuiLayoutOutInterface &outInterface,
    UrControlGuiLayout &layout) {
  layout._map.create(cfg.mapRsrcId);

  if (ErrorCode::SUCCESS != initButtonHandler(cfg, outInterface, layout)) {
    LOGERR("Error, initButtonHandler() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlGuiLayoutInitHelper::initButtonHandler(
    [[maybe_unused]]const UrControlGuiLayoutConfig &cfg,
    [[maybe_unused]]const UrControlGuiLayoutOutInterface& outInterface,
    [[maybe_unused]]UrControlGuiLayout &layout) {

//  constexpr size_t moveButtonsCount = 3;
//  if (moveButtonsCount != baseCfg.moveButtonsRsrcIds.size()) {
//    LOGERR("Error, moveButtonsRsrcIds.size() is: %zu, while it should be "
//           "exactly: %zu", baseCfg.moveButtonsRsrcIds.size(),
//           moveButtonsCount);
//    return ErrorCode::FAILURE;
//  }
//
//  MoveButtonConfig moveButtonCfg;
//  moveButtonCfg.infoTextFontId = baseCfg.moveButtonInfoTextFontId;
//  const std::array<Point, moveButtonsCount> buttonsPos {
//    Point(235, 195), Point(85, 330), Point(385, 330) };
//  const std::array<Point, moveButtonsCount> buttonsInfoTextPos {
//    Point(270, 335), Point(80, 465), Point(380, 465) };
//  const std::array<std::string, moveButtonsCount> buttonsInfoTextContent {
//      "Move", "Rotate Left", "Rotate Right" };
//  const std::array<MoveType, moveButtonsCount> buttonsMoveType {
//      MoveType::FORWARD, MoveType::ROTATE_LEFT, MoveType::ROTATE_RIGHT };
//
//  for (size_t i = 0; i < moveButtonsCount; ++i) {
//    moveButtonCfg.rsrcId = baseCfg.moveButtonsRsrcIds[i];
//    moveButtonCfg.startPos = buttonsPos[i];
//    moveButtonCfg.moveType = buttonsMoveType[i];
//    moveButtonCfg.infoTextContent = buttonsInfoTextContent[i];
//    moveButtonCfg.infoTextPos = buttonsInfoTextPos[i];
//
//    cfg.moveButtonsCfgs.emplace_back(moveButtonCfg);
//  }
//
//  cfg.settingsBtnCfg.pos = Point(480, 80);
//  cfg.settingsBtnCfg.rsrcId = baseCfg.settingsButtonRsrcId;
//
//  cfg.helpBtnCfg.pos = Point(480, 160);
//  cfg.helpBtnCfg.rsrcId = baseCfg.helpButtonRsrcId;
//
//  if (ErrorCode::SUCCESS !=
//      layout._controller.init(cfg, uiControllerOutInterface)) {
//    LOGERR("Error in _controller.init()");
//    return ErrorCode::FAILURE;
//  }

  return ErrorCode::SUCCESS;
}

