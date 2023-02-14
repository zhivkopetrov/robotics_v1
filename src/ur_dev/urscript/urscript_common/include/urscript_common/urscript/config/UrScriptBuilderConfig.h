#ifndef URSCRIPT_URSCRIPTBUILDERCONFIG_H_
#define URSCRIPT_URSCRIPTBUILDERCONFIG_H_

//System headers

//Other libraries headers

//Own components headers
#include "urscript_common/defines/UrScriptDefines.h"

//Forward declarations

struct UrScriptBuilderConfig {
  GripperType gripperType = GripperType::SIMULATION;
  std::string gripperDefinitionFolder;
};

#endif /* URSCRIPT_URSCRIPTBUILDERCONFIG_H_ */
