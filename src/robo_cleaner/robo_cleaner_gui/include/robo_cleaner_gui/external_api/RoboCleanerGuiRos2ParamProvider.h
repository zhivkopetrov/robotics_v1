#ifndef ROBO_CLEANER_GUI_ROBOCLEANERGUIROS2PARAMPROVIDER_H_
#define ROBO_CLEANER_GUI_ROBOCLEANERGUIROS2PARAMPROVIDER_H_

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

struct RoboCleanerGuiRos2Params {
  Rectangle guiWindow;
  uint32_t engineTargetFps { };
  RendererPolicy rendererExecutionPolicy;
  RendererFlagsMask rendererFlagsMask;
  uint32_t resLoadingThreadsNum { };
  FboOptimization fboOptimization = FboOptimization::ENABLED;
  Ros2CommunicatorConfig ros2CommunicatorConfig;
  int32_t levelId { };
  FogOfWarStatus fogOfWarStatus = FogOfWarStatus::ENABLED;

  void print() const;
  void validate();
};

class RoboCleanerGuiRos2ParamProvider : public rclcpp::Node {
public:
  RoboCleanerGuiRos2ParamProvider();

  RoboCleanerGuiRos2Params getParams();

private:
  RoboCleanerGuiRos2Params _params;
};

#endif /* ROBO_CLEANER_GUI_ROBOCLEANERGUIROS2PARAMPROVIDER_H_ */
