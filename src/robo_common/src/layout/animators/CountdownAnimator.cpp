//Corresponding header
#include "robo_common/layout/animators/CountdownAnimator.h"

//System headers
#include <string>

//Other libraries headers
#include "utils/file_system/FileSystemUtils.h"
#include "utils/drawing/WidgetAligner.h"
#include "utils/Log.h"

//Own components headers

ErrorCode CountdownAnimator::init(const CountdownAnimatorConfig &cfg,
                                  const ShutdownGameCb &shutdownGameCb,
                                  const TakeScreenshotCb &takeScreenshotCb) {
  if (nullptr == shutdownGameCb) {
    LOGERR("Error, nullptr provided for ShutdownGameCb");
    return ErrorCode::FAILURE;
  }
  _shutdownGameCb = shutdownGameCb;

  if (nullptr == takeScreenshotCb) {
    LOGERR("Error, nullptr provided for TakeScreenshotCb");
    return ErrorCode::FAILURE;
  }
  _takeScreenshotCb = takeScreenshotCb;

  _projectName = cfg.projectName;
  _timerId = cfg.timerId;
  _countdownSecondsLeft = cfg.countdownSeconds;

  createTexts(cfg);

  return ErrorCode::SUCCESS;
}

void CountdownAnimator::draw() const {
  if (!_isActive) {
    return;
  }

  _descriptionText.draw();
  _countdownText.draw();
  _screenshotDescriptionText.draw();
  _screenshotLocationText.draw();
}

void CountdownAnimator::startAnim() {
  if (_isActive) {
    LOGERR("Error, CountdownAnimator is already active");
    return;
  }

  _isActive = true;

  captureScreenshot();
  _screenshotDescriptionText.show();
  _screenshotLocationText.show();

  constexpr int64_t interval = 1000;
  startTimer(interval, _timerId, TimerType::PULSE);
}

void CountdownAnimator::setUserName(const std::string &userName) {
  _user = userName;
}

void CountdownAnimator::onTimeout(const int32_t timerId) {
  if (timerId == _timerId) {
    processAnim();
  } else {
    LOGERR("Error, received unsupported timerId: %d", timerId);
  }
}

void CountdownAnimator::processAnim() {
  --_countdownSecondsLeft;
  if (0 == _countdownSecondsLeft) {
    //TODO add user params to robo collector controller yaml file
    //TODO create a message with user data
    //TODO authenticate user to server from robo collector controller
    //TODO do this for all 3 robo games

    stopTimer(_timerId);
    _shutdownGameCb();
    return;
  }

  _countdownText.setText(std::to_string(_countdownSecondsLeft).c_str());
}

void CountdownAnimator::createTexts(const CountdownAnimatorConfig &cfg) {
  _descriptionText.create(cfg.countdownFontId, "System shutting down in ... ",
      Colors::RED);
  const Rectangle descrTextRect = _descriptionText.getFrameRect();
  constexpr int32_t offset = 10;
  const int32_t maxTextWidth = cfg.containerDimensions.w / 2;

  Point pos;
  pos.x = cfg.containerDimensions.x + offset;
  pos.x -= 120; //manual modification
  pos.y = WidgetAligner::getAlignedY(descrTextRect.h, cfg.containerDimensions.y,
      cfg.containerDimensions.h, WidgetAlignment::LOWER_LEFT,
      Margin(30, 30, 30, 30));

  _descriptionText.setPosition(pos);
  const int32_t descrTextX = pos.x;

  pos.x += (descrTextRect.w + offset);
  _countdownText.create(cfg.countdownFontId,
      std::to_string(cfg.countdownSeconds).c_str(), Colors::RED, pos);

  pos.x = descrTextX;
  pos.y += (_countdownText.getFrameHeight() + offset);
  _screenshotDescriptionText.create(cfg.screenshotFontId,
      "Saving screenshot to [screenshots] dir under working directory:",
      Colors::RED, pos);
  _screenshotDescriptionText.hide();
  _screenshotDescriptionText.activateScaling();
  _screenshotDescriptionText.setMaxScalingWidth(maxTextWidth);

  const Rectangle screnshotDescrRect =
      _screenshotDescriptionText.getFrameRect();
  pos.y += (screnshotDescrRect.h + offset);
  _screenshotLocationText.create(cfg.screenshotFontId, " ", Colors::RED, pos);
  _screenshotLocationText.hide();
  _screenshotLocationText.activateScaling();
  _screenshotLocationText.setMaxScalingWidth(maxTextWidth);
}

void CountdownAnimator::captureScreenshot() {
  constexpr auto screenshotFolder = "screenshots/";
  const std::string workingDir = FileSystemUtils::getCurrentWorkingDirectory();
  const std::string screenShotDir = workingDir + "/" + screenshotFolder;
  if (!FileSystemUtils::isDirectoryPresent(screenshotFolder)) {
    if (ErrorCode::SUCCESS != FileSystemUtils::createDirectoryRecursive(
            screenShotDir)) {
      LOGERR("Error, FileSystemUtils::createDirectoryRecursive() failed for "
             "directory: [%s]", screenShotDir.c_str());
      return;
    }
  }

  constexpr int32_t bufferSize = 31;
  time_t timer;
  char buffer[bufferSize];
  time(&timer);
  const tm *tm_info = localtime(&timer);
  strftime(buffer, bufferSize, "[%Y-%m-%d]_[%H:%M:%S]", tm_info);

  std::string fileName = screenshotFolder;
  fileName.append(_user).append("_").append(_projectName).append("_").append(
      buffer).append(".png");

  _screenshotLocationText.setText(fileName.c_str());
  _screenshotLocationText.setScaledWidth(
      _screenshotLocationText.getFrameWidth());

  constexpr int32_t quality = 100;
  _takeScreenshotCb(fileName.c_str(), ScreenshotContainer::PNG, quality);
}

