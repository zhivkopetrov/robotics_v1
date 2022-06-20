#ifndef ROBO_COMMON_COUNTDOWNANIMATOR_H_
#define ROBO_COMMON_COUNTDOWNANIMATOR_H_

//System headers
#include <cstdint>
#include <functional>

//Other libraries headers
#include "manager_utils/time/TimerClient.h"
#include "manager_utils/drawing/Text.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

struct CountdownAnimatorConfig {
  Rectangle containerDimensions;
  uint64_t fontId { };
  int32_t countdownSeconds = 10;
  int32_t timerId { };
};

class CountdownAnimator final : public TimerClient {
public:
  ErrorCode init(const CountdownAnimatorConfig &cfg,
                 const ShutdownGameCb &shutdownGameCb);

  void draw() const;

  void startAnim();

private:
  void onTimeout(const int32_t timerId) override;

  void processAnim();

  void createTexts(const CountdownAnimatorConfig &cfg);

  ShutdownGameCb _shutdownGameCb;

  Text _descriptionText;
  Text _countdownText;

  int32_t _timerId { };
  int32_t _countdownSecondsLeft { };
  bool _isActive = false;
};

#endif /* ROBO_COMMON_COUNTDOWNANIMATOR_H_ */
