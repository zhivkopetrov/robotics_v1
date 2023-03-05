#ifndef URSCRIPT_COMMON_URSCRIPTBUILDER_H_
#define URSCRIPT_COMMON_URSCRIPTBUILDER_H_

//System headers
#include <string_view>
#include <string>

//Other libraries headers

//Own components headers
#include "urscript_common/urscript/config/UrScriptBuilderConfig.h"
#include "urscript_common/urscript/UrScriptCommandContainer.h"
#include "utils/ErrorCode.h"

//Forward declarations

class UrScriptBuilder {
public:
  ErrorCode init(const UrScriptBuilderConfig& cfg);

  //transfers stored commands from commandsContainer and 
  //embbeds them into a UrScriptPayload
  UrScriptPayload construct(
    std::string_view methodName, 
    UrScriptCommandContainer& commandContainer) const;

private:
  UrScriptPayload _gripperDefinitionsPayload;

  //TODO remove when URSim docker image is extented to work with Robotiq URScripts
  GripperType _gripperType = GripperType::SIMULATION;
};

#endif /* URSCRIPT_COMMON_URSCRIPTBUILDER_H_ */