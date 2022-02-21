//Corresponding header
#include "robo_collector_controller/layout/RoboCollectorControllerLayout.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_controller/layout/helpers/RoboCollectorControllerLayoutInterfaces.h"
#include "robo_collector_controller/layout/helpers/RoboCollectorControllerLayoutInitHelper.h"

int32_t RoboCollectorControllerLayout::init(
    const RoboCollectorControllerLayoutConfig &cfg,
    const RoboCollectorControllerLayoutOutInterface &outInterface,
    RoboCollectorControllerLayoutInterface &interface) {
  if (SUCCESS !=
      RoboCollectorControllerLayoutInitHelper::init(cfg, outInterface, *this)) {
    LOGERR("Error, RoboCollectorControllerLayoutInitHelper::init() failed");
    return FAILURE;
  }

  produceInterface(interface);
  return SUCCESS;
}

void RoboCollectorControllerLayout::deinit() {

}

void RoboCollectorControllerLayout::draw() const {
  _map.draw();

  if (_controller.isEnabled()) {
    _controller.draw();
  }
}

void RoboCollectorControllerLayout::handleEvent(const InputEvent &e) {
  if (_controller.isEnabled()) {
    _controller.handleEvent(e);
  }
}

void RoboCollectorControllerLayout::produceInterface(
    RoboCollectorControllerLayoutInterface& interface) {
  interface.enablePlayerInputCb =
      std::bind(&RoboCollectorUiController::unlockInput, &_controller);
}

