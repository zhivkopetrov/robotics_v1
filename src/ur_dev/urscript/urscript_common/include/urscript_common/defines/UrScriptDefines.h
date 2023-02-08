#ifndef URSCRIPT_COMMON_URSCRIPTDEFINES_H_
#define URSCRIPT_COMMON_URSCRIPTDEFINES_H_

//System headers
#include <string>

//Other libraries headers

//Own components headers

//Forward declarations

using UrScriptPayload = std::string;

enum Ur10eRobotJoints {
  BASE_JOINT_IDX,
  SHOULDER_JOINT_IDX,
  ELBOW_JOINT_IDX,
  WRIST_1_JOINT_IDX,
  WRIST_2_JOINT_IDX,
  WRIST_3_JOINT_IDX,

  UR_10E_JOINTS_COUNT
};

#endif /* URSCRIPT_COMMON_URSCRIPTDEFINES_H_ */