#ifndef URSCRIPT_COMMON_URSCRIPTCOMMANDCONTAINER_H_
#define URSCRIPT_COMMON_URSCRIPTCOMMANDCONTAINER_H_

//System headers
#include <vector>
#include <memory>

//Other libraries headers

//Own components headers
#include "urscript_common/motion/MotionStructs.h"

//Forward declarations

using UrscriptContainerCommands = std::vector<std::unique_ptr<UrScriptCommandBase>>;

class UrscriptCommandContainer {
public:
  UrscriptCommandContainer& addCommand(
      std::unique_ptr<UrScriptCommandBase> cmd) {
    _cmds.push_back(std::move(cmd));
    return *this;
  }

  UrscriptContainerCommands transferStoredCommands() {
    return std::move(_cmds);
  }

private:
  UrscriptContainerCommands _cmds;
};

#endif /* URSCRIPT_COMMON_URSCRIPTCOMMANDCONTAINER_H_ */