#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMINITHELPER_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMINITHELPER_H_

//System headers
#include <cstdint>
#include <any>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class UrControlBloom;
struct UrControlBloomLayoutConfig;
struct UrContolBloomExternalBridgeConfig;
struct UrControlCommonLayoutInterface;
struct BloomMotionSequenceConfig;

class UrControlBloomInitHelper {
public:
  UrControlBloomInitHelper() = delete;

  static ErrorCode init(const std::any &cfg, UrControlBloom &bloom);

private:
  static ErrorCode initLayout(
      const UrControlBloomLayoutConfig &cfg,
      UrControlCommonLayoutInterface &layoutInterface, //out param
      UrControlBloom &bloom);

  static ErrorCode initDashboardHelper(
      const UrControlCommonLayoutInterface &layoutInterface, 
      UrControlBloom &bloom);

  static ErrorCode initUrControlBloomExternalBridge(
      const UrContolBloomExternalBridgeConfig &cfg,
      const UrControlCommonLayoutInterface &layoutInterface, 
      UrControlBloom &bloom);
      

  static ErrorCode initMotionExecutor(
    const BloomMotionSequenceConfig &cfg, UrControlBloom &bloom);
  
  static ErrorCode initStateMachine(UrControlBloom &bloom);
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMINITHELPER_H_ */
