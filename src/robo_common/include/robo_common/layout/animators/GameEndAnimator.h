#ifndef ROBO_COMMON_GAMEENDANIMATOR_H_
#define ROBO_COMMON_GAMEENDANIMATOR_H_

//System headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"
#include "utils/ErrorCode.h"

//Forward declarations

class GameEndAnimator {
public:
  ErrorCode init(const ShutdownGameCb& shutdownGameCb);
  void draw() const;

  void startGameWonAnim();
  void startGameLostAnim();
  void startAchievementWonAnim(Achievement achievement);

private:
  ShutdownGameCb _shutdownGameCb;
};

#endif /* ROBO_COMMON_GAMEENDANIMATOR_H_ */
