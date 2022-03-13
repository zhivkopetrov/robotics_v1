#ifndef ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUT_H_
#define ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUT_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "robo_collector_common/layout/controller/RoboCollectorUiController.h"

//Own components headers

//Forward declarations
class InputEvent;
struct RoboCollectorControllerLayoutConfig;
struct RoboCollectorControllerLayoutOutInterface;
struct RoboCollectorControllerLayoutInterface;

class RoboCollectorControllerLayout {
public:
  friend class RoboCollectorControllerLayoutInitHelper;

  ErrorCode init(const RoboCollectorControllerLayoutConfig &cfg,
                 const RoboCollectorControllerLayoutOutInterface &outInterface,
                 RoboCollectorControllerLayoutInterface& interface);
  void deinit();
  void draw() const;
  void handleEvent(const InputEvent &e);

private:
  void produceInterface(RoboCollectorControllerLayoutInterface& interface);

  Image _map;
  RoboCollectorUiController _controller;
};

#endif /* ROBO_COLLECTOR_CONTROLLER_ROBOCOLLECTORCONTROLLERLAYOUT_H_ */
