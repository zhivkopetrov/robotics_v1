#ifndef ROBO_COMMON_ENDSCREENAPPEARANIMATOR_H_
#define ROBO_COMMON_ENDSCREENAPPEARANIMATOR_H_

//System headers
#include <cstdint>
#include <array>
#include <functional>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"
#include "manager_utils/time/TimerClient.h"
#include "manager_utils/drawing/Image.h"
#include "manager_utils/drawing/Text.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_common/layout/animators/config/GameEndAnimatorConfig.h"

//Forward declarations

using OnAppearAnimFinish = std::function<void()>;

class EndScreenAppearAnimator final : public TimerClient {
public:
  ErrorCode init(const GameEndAnimatorConfig &cfg,
                 const OnAppearAnimFinish &onAppearAnimFinish,
                 Fbo *finalScreenFbo);

  void draw() const;

  void setUserData(const UserData &userData);

  void startAnim(EndGameOutcome outcome);

private:
  void createVisuals(const GameEndAnimatorConfig &cfg, Fbo *finalScreenFbo);
  void createBgrImage(const GameEndAnimatorConfig &cfg, Fbo *finalScreenFbo);
  void createUserDataTexts(const GameEndAnimatorConfig &cfg,
                           Fbo *finalScreenFbo);
  void populateEndGameOutcomeText(EndGameOutcome outcome);

  void onTimeout(const int32_t timerId) override;

  void processFadeAnim();
  void processExpandAnim();

  void updateFinalScreenFbo(Fbo *finalScreenFbo);

  struct FadeAnimData {
    int32_t currOpacity = ZERO_OPACITY;
    int32_t fadeStep = 8;
    int32_t timerId { };
    const int32_t endOpacity = 200;
  };

  struct ExpandAnimData {
    int32_t timerId { };
    int32_t widthStep { };
    int32_t heighStep { };
    int32_t currStepIdx { };
    Rectangle expandedDimensions;
    const int32_t totalSteps = 25;
    const int32_t lastStepIdx = totalSteps - 1;
  };

  enum InternalDefines {
    USER_TEXTS_COUNT = 3
  };

  Fbo _blackBgrFbo;
  Image _bgrImg;
  std::array<Text, USER_TEXTS_COUNT> _userTexts;
  Text _endGameOutcomeText;
  Fbo *_finalScreenFbo = nullptr;

  FadeAnimData _fadeAnimData;
  ExpandAnimData _expandAnimData;
  UserData _userData;

  OnAppearAnimFinish _onAppearAnimFinish;
};

#endif /* ROBO_COMMON_ENDSCREENAPPEARANIMATOR_H_ */
