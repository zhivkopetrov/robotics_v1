#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMROS2PARAMPROVIDER_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMROS2PARAMPROVIDER_H_

//System headers
#include <string>

//Other libraries headers
#include <rclcpp/node.hpp>
#include "urscript_common/defines/UrScriptDefines.h"
#include "ros2_game_engine/communicator/config/Ros2CommunicatorConfig.h"
#include "utils/drawing/Rectangle.h"

//Own components headers
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

//Forward declarations

struct UrControlBloomRos2Params {
  Rectangle guiWindow;
  Ros2CommunicatorConfig ros2CommunicatorConfig;
  std::string robotIp;
  uint16_t robotInterfacePort {};
  GripperType gripperType = GripperType::SIMULATION;
  JengaEndStrategy jengaEndStrategy = JengaEndStrategy::SWAP_TOWERS;

  void print() const;
  void validate();
};

class UrControlBloomRos2ParamProvider : public rclcpp::Node {
public:
  UrControlBloomRos2ParamProvider();

  UrControlBloomRos2Params getParams();

private:
  UrControlBloomRos2Params _params;
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMROS2PARAMPROVIDER_H_ */
