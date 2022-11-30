#ifndef ROBO_MINER_GUI_ROBOMINERGUIROS2PARAMPROVIDER_H_
#define ROBO_MINER_GUI_ROBOMINERGUIROS2PARAMPROVIDER_H_

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>
#include "robo_common/layout/field/config/FogOfWarConfig.h"
#include "robo_common/defines/RoboCommonDefines.h"
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"
#include "sdl_utils/drawing/config/RendererConfig.h"
#include "utils/drawing/Rectangle.h"

//Own components headers

//Forward declarations

struct RoboMinerGuiRos2Params {
  Rectangle guiWindow;
  uint32_t engineTargetFps { };
  RendererPolicy rendererExecutionPolicy;
  RendererFlagsMask rendererFlagsMask;
  FboOptimization fboOptimization = FboOptimization::ENABLED;
  Ros2CommunicatorConfig ros2CommunicatorConfig;
  int32_t levelId { };
  FogOfWarStatus fogOfWarStatus = FogOfWarStatus::ENABLED;

  void print() const;
  void validate();
};

class RoboMinerGuiRos2ParamProvider : public rclcpp::Node {
public:
  RoboMinerGuiRos2ParamProvider();

  RoboMinerGuiRos2Params getParams();

private:
  RoboMinerGuiRos2Params _params;
};

#endif /* ROBO_MINER_GUI_ROBOMINERGUIROS2PARAMPROVIDER_H_ */
