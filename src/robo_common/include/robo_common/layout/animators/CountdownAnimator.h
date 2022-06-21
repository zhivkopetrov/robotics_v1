#ifndef ROBO_COMMON_COUNTDOWNANIMATOR_H_
#define ROBO_COMMON_COUNTDOWNANIMATOR_H_

//System headers
#include <cstdint>
#include <functional>
#include <string>

//Other libraries headers
#include "manager_utils/time/TimerClient.h"
#include "manager_utils/drawing/Text.h"
#include "sdl_utils/drawing/defines/DrawUtilityDefines.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

struct CountdownAnimatorConfig {
  std::string projectName = "project_name_not_set";
  Rectangle containerDimensions;
  uint64_t countdownFontId { };
  uint64_t screenshotFontId { };
  int32_t countdownSeconds = 10;
  int32_t timerId { };
};

class CountdownAnimator final : public TimerClient {
public:
  ErrorCode init(const CountdownAnimatorConfig &cfg,
                 const ShutdownGameCb &shutdownGameCb,
                 const TakeScreenshotCb &takeScreenshotCb);

  void draw() const;

  void startAnim();

  void setUserName(const std::string& userName);

private:
  void onTimeout(const int32_t timerId) override;

  void processAnim();

  void createTexts(const CountdownAnimatorConfig &cfg);

  void captureScreenshot();

  ShutdownGameCb _shutdownGameCb;
  TakeScreenshotCb _takeScreenshotCb;

  Text _descriptionText;
  Text _countdownText;
  Text _screenshotDescriptionText;
  Text _screenshotLocationText;

  std::string _user = "user_not_set";
  std::string _projectName;

  int32_t _timerId { };
  int32_t _countdownSecondsLeft { };
  bool _isActive = false;
};

#endif /* ROBO_COMMON_COUNTDOWNANIMATOR_H_ */
