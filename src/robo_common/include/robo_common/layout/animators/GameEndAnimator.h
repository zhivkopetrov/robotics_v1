#ifndef ROBO_COMMON_GAMEENDANIMATOR_H_
#define ROBO_COMMON_GAMEENDANIMATOR_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonFunctionalDefines.h"

//Forward declarations

class GameEndAnimator {
public:
  int32_t init(const ShutdownGameCb& shutdownGameCb);
  void draw() const;

  void startGameWonAnim();
  void startGameLostAnim();

private:
  ShutdownGameCb _shutdownGameCb;
};

#endif /* ROBO_COMMON_GAMEENDANIMATOR_H_ */
