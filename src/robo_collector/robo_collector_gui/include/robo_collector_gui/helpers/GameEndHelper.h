#ifndef ROBO_COLLECTOR_GUI_GAMEENDHELPER_H_
#define ROBO_COLLECTOR_GUI_GAMEENDHELPER_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers

//Own components headers

//Forward declarations

class GameEndHelper {
public:
  int32_t init();
  void draw() const;

  //TODO animate
  void gameWon();
  void gameLost();

private:
};

#endif /* ROBO_COLLECTOR_GUI_GAMEENDHELPER_H_ */
