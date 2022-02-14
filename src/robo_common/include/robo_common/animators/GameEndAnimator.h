#ifndef ROBO_COMMON_GAMEENDANIMATOR_H_
#define ROBO_COMMON_GAMEENDANIMATOR_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

class GameEndAnimator {
public:
  int32_t init();
  void draw() const;

  //TODO animate
  void gameWon();
  void gameLost();

private:
};

#endif /* ROBO_COMMON_GAMEENDANIMATOR_H_ */
