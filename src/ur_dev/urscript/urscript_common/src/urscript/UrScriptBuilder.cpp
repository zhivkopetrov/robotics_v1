//Corresponding header
#include "urscript_common/urscript/UrScriptBuilder.h"

//System headers
#include <sstream>

//Other libraries headers
#include "utils/log/Log.h"

//Own components headers
#include "urscript_common/urscript/UrScriptParser.h"

ErrorCode UrScriptBuilder::init(const UrScriptBuilderConfig& cfg) {
  std::vector<std::string> parsedScripts;
  const ErrorCode err = 
    UrScriptParser::parseScripts(cfg.gripperDefinitionFolder, parsedScripts);
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, UrScriptParser::parseScripts() failed");
    return ErrorCode::FAILURE;
  }

  constexpr size_t expectedParsedScriptsCount = 1;
  const size_t parsedScriptsCount = parsedScripts.size();
  if (expectedParsedScriptsCount != parsedScriptsCount) {
    LOGERR("Error, Scripts count missmatch. Scripts parsed: %zu vs "
           "Expected scripts count: %zu", parsedScriptsCount, 
           expectedParsedScriptsCount);
    return ErrorCode::FAILURE;
  }

  _gripperDefinitionsPayload = std::move(parsedScripts.front());

  //TODO remove when URSim docker image is extented to work with
  //     Robotiq URScripts
  setGripperTypeGlobally(cfg.gripperType);
  _gripperType = cfg.gripperType;

  return ErrorCode::SUCCESS;
}

UrScriptPayload UrScriptBuilder::construct(
    std::string_view methodName, 
    UrScriptCommandContainer& commandContainer) const {
  const UrScriptContainerCommands cmds = 
    commandContainer.transferStoredCommands();

  std::ostringstream ostr;
  ostr << "def " << methodName << "():\n";

  // if at least one command is of type GRIPPER - 
  // gripper definitions should be enabled
  if (GripperType::HARDWARE == _gripperType) {
    for (const auto& cmd : cmds) {
      if (UrScriptCommandType::GRIPPER == cmd->getCommandType()) {
        ostr << _gripperDefinitionsPayload;
        break;
      }
    }
  }

  for (const auto& cmd : cmds) {
    ostr << '\t' << cmd->construct() << '\n';
  }
  ostr << "end\n";

  return ostr.str();
}