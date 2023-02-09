#ifndef URSCRIPT_COMMON_URSCRIPTBUILDER_H_
#define URSCRIPT_COMMON_URSCRIPTBUILDER_H_

//System headers
#include <string_view>

//Other libraries headers

//Own components headers
#include "urscript_common/motion/MotionCommandContainer.h"

//Forward declarations

class UrScriptBuilder {
public:
  UrScriptBuilder() = delete;

  //transfers stored commands from commandsContainer and 
  //embbeds them into a UrScriptPayload
  static UrScriptPayload construct(
      std::string_view methodName, MotionCommandContainer& commandContainer);
};

#endif /* URSCRIPT_COMMON_URSCRIPTBUILDER_H_ */