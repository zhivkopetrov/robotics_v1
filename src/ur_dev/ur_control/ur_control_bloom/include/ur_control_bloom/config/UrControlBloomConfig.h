#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMCONFIG_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMCONFIG_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers

//Own components headers
#include "ur_control_bloom/layout/config/UrControlBloomLayoutConfig.h"
#include "ur_control_bloom/external_api/config/UrContolBloomExternalBridgeConfig.h"
#include "ur_control_bloom/motion/config/UrControlBloomMotionSequenceConfig.h"

//Forward declarations

struct UrControlBloomConfig {
  UrControlBloomLayoutConfig layoutCfg;
  UrContolBloomExternalBridgeConfig externalBridgeCfg;
  UrControlBloomMotionSequenceConfig motionSequenceCfg;
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMCONFIG_H_ */

