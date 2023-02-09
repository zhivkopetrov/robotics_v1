//Corresponding header
#include "urscript_common/motion/UrScriptBuilder.h"

//System headers
#include <sstream>

//Other libraries headers

//Own components headers

UrScriptPayload UrScriptBuilder::construct(
    std::string_view methodName, MotionCommandContainer& commandContainer) {
  const MotionContainerCommands cmds = 
    commandContainer.transferStoredCommands();

  std::ostringstream ostr;
  ostr << "def " << methodName << "():\n";
  for (const auto& cmd : cmds) {
    ostr << '\t' << cmd->construct() << '\n';
  }
  ostr << "end\n";

  return ostr.str();
}