#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMCONFIG_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMCONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "urscript_common/urscript/config/UrScriptBuilderConfig.h"

//Own components headers
#include "ur_control_bloom/layout/config/UrControlBloomLayoutConfig.h"
#include "ur_control_bloom/external_api/config/UrContolBloomExternalBridgeConfig.h"
#include "ur_control_bloom/motion/config/UrControlBloomMotionSequenceConfig.h"

//Forward declarations

struct UrControlBloomConfig {
  UrControlBloomLayoutConfig layoutCfg;
  UrContolBloomExternalBridgeConfig externalBridgeCfg;
  UrControlBloomMotionSequenceConfig motionSequenceCfg;
  UrScriptBuilderConfig urScriptBuilderCfg;
  std::string stateFilePath;
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMCONFIG_H_ */

