#ifndef ROBO_COLLECTOR_GUI_ROBOT_H_
#define ROBO_COLLECTOR_GUI_ROBOT_H_

//C system headers

//C++ system headers
#include <cstdint>

//Other libraries headers
#include "manager_utils/drawing/Image.h"

//Own components headers

//Forward declarations
class InputEvent;

struct RobotCfg {
  Point startPos;
  uint64_t rsrcId = 0;
  int32_t frameId = 0;
};

class Robot {
public:
  int32_t init(const RobotCfg& cfg);

  void draw() const;

  void handleEvent(const InputEvent& e);

private:
  Image _img;
};

#endif /* ROBO_COLLECTOR_GUI_ROBOT_H_ */
