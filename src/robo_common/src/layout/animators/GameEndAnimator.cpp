//Corresponding header
#include "robo_common/layout/animators/GameEndAnimator.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t GameEndAnimator::init() {
  return SUCCESS;
}

void GameEndAnimator::draw() const {

}

void GameEndAnimator::gameWon() {
  LOGG("You've Won");
}

void GameEndAnimator::gameLost() {
  LOGR("You've Lost");
}
