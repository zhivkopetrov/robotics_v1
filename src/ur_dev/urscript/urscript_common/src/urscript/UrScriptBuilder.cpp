//Corresponding header
#include "urscript_common/urscript/UrScriptBuilder.h"

//System headers
#include <sstream>

//Other libraries headers

//Own components headers

namespace {
constexpr auto GRIPPER_DEFINITIONS_PAYLOAD = 
  "# TODO populate on init from file";
}

UrScriptPayload UrScriptBuilder::construct(
    std::string_view methodName, UrScriptCommandContainer& commandContainer) {
  const UrScriptContainerCommands cmds = 
    commandContainer.transferStoredCommands();

  std::ostringstream ostr;
  ostr << "def " << methodName << "():\n";

  // if at least one command is of type GRIPPER - 
  // gripper definitions should be enabled
  for (const auto& cmd : cmds) {
    if (UrScriptCommandType::GRIPPER == cmd->getCommandType()) {
      ostr << GRIPPER_DEFINITIONS_PAYLOAD;
      break;
    }
  }

  for (const auto& cmd : cmds) {
    ostr << '\t' << cmd->construct() << '\n';
  }
  ostr << "end\n";

  return ostr.str();
}