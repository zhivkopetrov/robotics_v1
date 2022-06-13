//Corresponding header
#include "robo_common/layout/animators/GameEndAnimator.h"

//System headers

//Other libraries headers
#include "utils/data_type/EnumClassUtils.h"
#include "utils/Log.h"

//Own components headers

ErrorCode GameEndAnimator::init(const ShutdownGameCb& shutdownGameCb) {
  if (nullptr == shutdownGameCb) {
    LOGERR("Error, nullptr provided for ShutdownGameCb");
    return ErrorCode::FAILURE;
  }
  _shutdownGameCb = shutdownGameCb;

  return ErrorCode::SUCCESS;
}

void GameEndAnimator::draw() const {

}

void GameEndAnimator::startGameWonAnim() {
  LOGG("You've Won");

  //TODO add animation and invoke shutdownGameCb on animation end
//  _shutdownGameCb();
}

void GameEndAnimator::startGameLostAnim() {
  LOGR("You've Lost");

  //TODO add animation and invoke shutdownGameCb on animation end
//  _shutdownGameCb();
}

void GameEndAnimator::startAchievementWonAnim(Achievement achievement) {
  switch (achievement) {
  case Achievement::SINGLE_STAR:
    LOGG("You've Won a Single Star");
    break;
  case Achievement::DOUBLE_STAR:
    LOGG("You've Won a Double Star");
    break;
  case Achievement::TRIPLE_STAR:
    LOGG("You've Won a Tripple Star");
    break;
  default:
    LOGERR("Error, received unsupported Achievement value: %d",
        getEnumValue(achievement));
    break;
  }
}

