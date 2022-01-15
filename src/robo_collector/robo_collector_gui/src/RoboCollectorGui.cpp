//Corresponding header
#include "robo_collector_gui/RoboCollectorGui.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_collector_gui/config/RoboCollectorGuiConfig.h"

int32_t RoboCollectorGui::init(const std::any& cfg) {
  try {
      const auto& gameCfg = std::any_cast<const RoboCollectorGuiConfig&>(cfg);
      if (SUCCESS != _field.init(gameCfg.fieldCfg)) {
        LOGERR("Error in _field.init()");
        return FAILURE;
      }
  }
  catch(const std::bad_any_cast& e) {
      LOGERR("std::any_cast<GuiConfig&> failed, %s", e.what());
      return FAILURE;
  }

  return SUCCESS;
}

void RoboCollectorGui::deinit() {

}

void RoboCollectorGui::draw() const {
  _field.draw();
}

void RoboCollectorGui::handleEvent(const InputEvent &e) {
  _field.handleEvent(e);
}

