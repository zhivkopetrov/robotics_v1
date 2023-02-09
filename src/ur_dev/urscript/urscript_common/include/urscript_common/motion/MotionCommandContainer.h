#ifndef URSCRIPT_COMMON_COMMANDMOTIONCONTAINER_H_
#define URSCRIPT_COMMON_COMMANDMOTIONCONTAINER_H_

//System headers
#include <vector>
#include <memory>

//Other libraries headers

//Own components headers
#include "urscript_common/motion/MotionStructs.h"

//Forward declarations

using MotionContainerCommands = std::vector<std::unique_ptr<MotionCommandBase>>;

class MotionCommandContainer {
public:
  MotionCommandContainer& addCommand(std::unique_ptr<MotionCommandBase> cmd) {
    _cmds.push_back(std::move(cmd));
    return *this;
  }

  MotionContainerCommands transferStoredCommands() {
    return std::move(_cmds);
  }

private:
  MotionContainerCommands _cmds;
};

#endif /* URSCRIPT_COMMON_COMMANDMOTIONCONTAINER_H_ */