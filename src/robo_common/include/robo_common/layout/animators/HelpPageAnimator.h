#ifndef ROBO_COMMONHELPPAGEANIMATOR_H_
#define ROBO_COMMONHELPPAGEANIMATOR_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/time/TimerClient.h"
#include "manager_utils/drawing/Fbo.h"
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
struct HelpPageAnimatorConfig;

class HelpPageAnimator final : public TimerClient {
public:
  ErrorCode init(const HelpPageAnimatorConfig &cfg);

  void toggleStatus();

  void draw() const;

private:
  void onTimeout(const int32_t timerId) override;

  void createBlackBgrFbo(const Rectangle &dimensions);
  void createHelpPageFbo(const HelpPageAnimatorConfig &cfg);

  void processShowAnim();
  void processHideAnim();

  void changeAnimDirection();

  enum class AnimType {
    SHOW, HIDE
  };

  struct AnimData {
    int32_t timerId { };
    int32_t currStepIdx { };
    int32_t moveStep { };
    const int32_t totalSteps = 100;
    const int32_t lastShowStepIdx = totalSteps - 1;
    const int32_t lastHideStepIdx = 0;

    const int32_t endOpacity = 200;
    const int32_t opacityStep = endOpacity / totalSteps;
    int32_t currOpacity = ZERO_OPACITY;
  };

  Fbo _helpPageFbo;
  Fbo _blackBgrFbo;

  AnimData _animData;
  AnimType _currAnimType = AnimType::SHOW;

  bool _isActive = false;
};

#endif /* ROBO_COMMONHELPPAGEANIMATOR_H_ */
