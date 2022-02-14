//Corresponding header
#include "robo_common/helpers/GameEndHelper.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t GameEndHelper::init() {
  return SUCCESS;
}

void GameEndHelper::draw() const {

}

void GameEndHelper::gameWon() {
  LOGG("You've Won");
}

void GameEndHelper::gameLost() {
  LOGR("You've Lost");
}
