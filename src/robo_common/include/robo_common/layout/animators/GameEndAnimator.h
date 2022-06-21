#ifndef ROBO_COMMON_GAMEENDANIMATOR_H_
#define ROBO_COMMON_GAMEENDANIMATOR_H_

//System headers
#include <cstdint>
#include <set>

//Other libraries headers
#include "manager_utils/drawing/Fbo.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "robo_common/layout/animators/EndScreenAppearAnimator.h"
#include "robo_common/layout/animators/CountdownAnimator.h"

//Forward declarations

using IsAchievementAnimatorActive = std::function<bool()>;
using StartEndGameSequence = std::function<void(const std::set<Achievement>&)>;

struct GameEndAnimatorOutInterface {
  ShutdownGameCb shutdownGameCb;
  StartAchievementWonAnimCb startAchievementWonAnimCb;
  IsAchievementAnimatorActive isAchievementAnimatorActive;
  StartEndGameSequence startEndGameSequence;
  TakeScreenshotCb takeScreenshotCb;
};

class GameEndAnimator {
public:
  ErrorCode init(const GameEndAnimatorConfig& cfg,
                 const GameEndAnimatorOutInterface& outInterface);
  void draw() const;

  void startGameWonAnim();
  void startGameLostAnim();
  void startAchievementWonAnim(Achievement achievement);
  void setUserData(const UserData& userData);

  void onAchievementWonAnimFinish(Achievement achievement);

private:
  ErrorCode initOutInterface(const GameEndAnimatorOutInterface& outInterface);

  void onAppearAnimFinish();

  void createFinalScreenFbo(const GameEndAnimatorConfig& cfg);

  GameEndAnimatorOutInterface _outInterface;

  EndScreenAppearAnimator _appearAnimator;
  CountdownAnimator _countdownAnimator;
  Fbo _finalScreenFbo;

  std::set<Achievement> _wonAchievements;

  bool _isActive = false;
  bool _hasPendingEndStatus = false;
  EndGameOutcome _pendingGameOutcome = EndGameOutcome::WIN;
};

#endif /* ROBO_COMMON_GAMEENDANIMATOR_H_ */
