#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMINITHELPER_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMINITHELPER_H_

//System headers
#include <cstdint>
#include <any>
#include <string>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations
class UrControlBloom;
struct UrControlBloomConfig;
struct UrControlBloomLayoutConfig;
struct UrContolBloomExternalBridgeConfig;
struct UrControlCommonLayoutInterface;
struct UrControlBloomMotionSequenceConfig;
struct UrScriptBuilderConfig;

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

  static ErrorCode initUrScriptBuilder(
    const UrScriptBuilderConfig &cfg, UrControlBloom &bloom);

  static ErrorCode initStateFileHandler(
    const std::string &filePath, UrControlBloom &bloom);
      
  static ErrorCode initMotionExecutor(
    const UrControlBloomMotionSequenceConfig &cfg, UrControlBloom &bloom);
  
  static ErrorCode initStateMachine(UrControlBloom &bloom);
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMINITHELPER_H_ */
