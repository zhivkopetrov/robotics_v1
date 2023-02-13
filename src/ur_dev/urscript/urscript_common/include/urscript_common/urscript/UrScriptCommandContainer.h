#ifndef URSCRIPT_COMMON_URSCRIPTCOMMANDCONTAINER_H_
#define URSCRIPT_COMMON_URSCRIPTCOMMANDCONTAINER_H_

//System headers
#include <vector>
#include <memory>

//Other libraries headers

//Own components headers
#include "urscript_common/motion/MotionStructs.h"
#include "urscript_common/gripper/GripperStructs.h"

//Forward declarations

using UrScriptContainerCommands = std::vector<std::unique_ptr<UrScriptCommand>>;

class UrScriptCommandContainer {
public:
  UrScriptCommandContainer& addCommand(std::unique_ptr<UrScriptCommand> cmd) {
    _cmds.push_back(std::move(cmd));
    return *this;
  }

  UrScriptContainerCommands transferStoredCommands() {
    return std::move(_cmds);
  }

private:
  UrScriptContainerCommands _cmds;
};

#endif /* URSCRIPT_COMMON_URSCRIPTCOMMANDCONTAINER_H_ */