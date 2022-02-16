#ifndef ROBO_CLEANER_GUI_ROBOCLEANERBUILDER_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERBUILDER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <memory>

//Other libraries headers

//Own components headers
#include "robo_cleaner_gui/RoboCleanerGui.h"

//Forward declarations
class Ros2Communicator;

class RoboCleanerBuilder {
public:
  RoboCleanerBuilder() = delete;

  static std::unique_ptr<RoboCleanerGui> createRoboCleanerGui(
      const std::unique_ptr<Ros2Communicator>& communicator);
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERBUILDER_H_ */
