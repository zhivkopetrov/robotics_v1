#ifndef ROBO_COMMON_ACHIEVEMENTANIMATOR_H_
#define ROBO_COMMON_ACHIEVEMENTANIMATOR_H_

//System headers
#include <cstdint>
#include <functional>
#include <queue>
#include <set>

//Other libraries headers
#include "manager_utils/time/TimerClient.h"
#include "manager_utils/drawing/Image.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"
#include "robo_common/layout/animators/config/AchievementAnimatorConfig.h"

using OnAchievementWonAnimFinished = std::function<void(Achievement)>;

class AchievementAnimator final : public TimerClient {
public:
  ErrorCode init(
      const AchievementAnimatorConfig &cfg,
      const OnAchievementWonAnimFinished &onAchievementWonAnimFinished);

  void draw() const;

  void startAnim(Achievement achievement);

  void startEndGameSequence(const std::set<Achievement>& achievements);

  bool isActive() const;

private:
  void onTimeout(const int32_t timerId) override;

  void processAnim();

  void configureAnim(Achievement achievement);

  void consumeProcessedAchievement();

  enum class AnimDir {
    FORWARD, BACKWARD
  };

  struct AnimData {
    int32_t currStepIdx { };
    int32_t currOpacity { };
    int32_t timerId { };
    Rectangle allStarsDimensions;
    AnimDir dir = AnimDir::FORWARD;
    Point moveOffset;
    const int32_t moveStep = 8;
    const int32_t totalSteps = 17;
    const int32_t lastStepIdx = totalSteps - 1;
    const int32_t opacityStep = FULL_OPACITY / totalSteps;
  };

  Image _allStars;
  Image _singleStar;

  AnimData _animData;

  std::queue<Achievement> _achievementsToProcess;

  OnAchievementWonAnimFinished _onAchievementWonAnimFinished;

  bool _isActive = false;
  bool _endGameSequenceStarted = false;
};

#endif /* ROBO_COMMON_ACHIEVEMENTANIMATOR_H_ */
