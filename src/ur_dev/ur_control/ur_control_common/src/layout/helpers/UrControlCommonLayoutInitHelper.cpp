//Corresponding header
#include "ur_control_common/layout/helpers/UrControlCommonLayoutInitHelper.h"

//System headers

//Other libraries headers
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_common/layout/config/UrControlCommonLayoutConfig.h"
#include "ur_control_common/layout/UrControlCommonLayout.h"
#include "ur_control_common/layout/helpers/UrControlCommonLayoutInterfaces.h"

ErrorCode UrControlCommonLayoutInitHelper::init(
    const UrControlCommonLayoutConfig &cfg,
    const UrControlCommonLayoutOutInterface &outInterface,
    UrControlCommonLayout &layout) {
  if (ErrorCode::SUCCESS != initStandaloneImages(cfg, layout)) {
    LOGERR("Error, initStandaloneImages() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initButtonHandler(cfg.buttonHandlerConfig,
          outInterface, layout)) {
    LOGERR("Error, initButtonHandler() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != layout.safetyModeVisuals.init(cfg.screenBoundary,
          cfg.robotModeVisualsFontRsrcId)) {
    LOGERR("Error, safetyModeVisuals.init() failed");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlCommonLayoutInitHelper::initStandaloneImages(
    const UrControlCommonLayoutConfig &cfg, UrControlCommonLayout &layout) {
  layout._map.create(cfg.mapRsrcId);

  Image &robot = layout._robot;
  robot.create(cfg.robotImgRrscId);
  const Rectangle robotRect = robot.getFrameRect();
  const Point pos = WidgetAligner::getPosition(robotRect.w, robotRect.h,
      cfg.screenBoundary, WidgetAlignment::LOWER_CENTER);
  robot.setPosition(pos);

  return ErrorCode::SUCCESS;
}

ErrorCode UrControlCommonLayoutInitHelper::initButtonHandler(
    const ButtonHandlerConfig &cfg,
    const UrControlCommonLayoutOutInterface &outInterface,
    UrControlCommonLayout &layout) {

  ButtonHandlerOutInterface btnOutInterface;
  btnOutInterface.publishURScriptCb = outInterface.publishURScriptCb;
  btnOutInterface.invokeDashboardServiceCb = 
    outInterface.invokeDashboardServiceCb;

  if (ErrorCode::SUCCESS != layout.buttonHandler.init(cfg, btnOutInterface)) {
    LOGERR("Error in _buttonHandler.init()");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

