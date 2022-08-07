//Corresponding header
#include "ur_control_gui/layout/helpers/UrControlGuiLayoutInitHelper.h"

//System headers

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_gui/layout/config/UrControlGuiLayoutConfig.h"
#include "ur_control_gui/layout/UrControlGuiLayout.h"
#include "ur_control_gui/layout/helpers/UrControlGuiLayoutInterfaces.h"

ErrorCode UrControlGuiLayoutInitHelper::init(
    const UrControlGuiLayoutConfig &cfg,
    const UrControlGuiLayoutOutInterface &outInterface,
    UrControlGuiLayout &layout) {
  if (ErrorCode::SUCCESS != initStandaloneImages(cfg, layout)) {
    LOGERR("Error, initStandaloneImages() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initButtonHandler(cfg.buttonHandlerConfig,
          outInterface, layout)) {
    LOGERR("Error, initButtonHandler() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlGuiLayoutInitHelper::initStandaloneImages(
    const UrControlGuiLayoutConfig &cfg, UrControlGuiLayout &layout) {
  layout._map.create(cfg.mapRsrcId);

  Image &robot = layout._robot;
  robot.create(cfg.robotImgRrscId);
  const Rectangle robotRect = robot.getFrameRect();
  const Point pos = WidgetAligner::getPosition(robotRect.w, robotRect.h,
      cfg.screenBoundary, WidgetAlignment::LOWER_CENTER);
  robot.setPosition(pos);

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlGuiLayoutInitHelper::initButtonHandler(
    const ButtonHandlerConfig &cfg,
    const UrControlGuiLayoutOutInterface &outInterface,
    UrControlGuiLayout &layout) {

  ButtonHandlerOutInterface btnOutInterface;
  btnOutInterface.publishURScriptCb = outInterface.publishURScriptCb;
  btnOutInterface.invokeDashboardCb = outInterface.invokeDashboardCb;

  if (ErrorCode::SUCCESS != layout._buttonHandler.init(cfg, btnOutInterface)) {
    LOGERR("Error in _buttonHandler.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

