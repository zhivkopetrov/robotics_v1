#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMDEFINES_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMDEFINES_H_

//System headers

//Other libraries headers

//Own components headers

//Forward declarations

namespace BloomState {

constexpr auto SECTION_NAME = "BloomState";
constexpr auto STATE_ENTRY_NAME = "State";

constexpr auto STATES_COUNT = 6;
  
constexpr auto INIT = "Init";
constexpr auto IDLE = "Idle";
constexpr auto BLOOM = "Bloom";
constexpr auto BLOOM_RECOVERY = "Bloom Recovery";
constexpr auto JENGA = "Jenga";
constexpr auto JENGA_RECOVERY = "Jenga Recovery";

} //namespace BloomState

namespace Motion {

enum MotionId {
  BLOOM_MOTION_ID,
  JENGA_MOTION_ID
};

constexpr auto BLOOM_MOTION_SEQUENCE_NAME = "BloomMotionSequence";
constexpr auto JENGA_MOTION_SEQUENCE_NAME = "JengaMotionSequence";

namespace Bloom {

constexpr auto SECTION_NAME = "BloomMotion";
constexpr auto HOLDING_OBJECT_ENTRY_NAME = "HoldingObject";

constexpr auto GRASP_NAME = "BloomGrasp";
constexpr auto TRANSPORT_AND_PLACE_NAME = "BloomTransportAndPlace";
constexpr auto RETURN_HOME_NAME = "BloomReturnHome";
constexpr auto RETRACT_AND_RETURN_HOME_NAME = "BloomRetractAndReturnHome";
constexpr auto RETURN_HOME_AND_OPEN_GRIPPER_NAME = "BloomReturnHomeAndOpenGripper";

} //namespace Bloom

namespace Jenga {

constexpr auto SECTION_NAME = "JengaMotion";
constexpr auto HOLDING_OBJECT_ENTRY_NAME = "HoldingObject";
constexpr auto CURRENT_OBJECT_IDX_ENTRY_NAME = "CurrentObjectIdx";
constexpr auto DIRECTION_ENTRY_NAME = "Direction";

constexpr auto GRASP_NAME = "JengaGrasp";
constexpr auto TRANSPORT_AND_PLACE_NAME = "JengaTransportAndPlace";
constexpr auto RETURN_HOME_NAME = "JengaReturnHome";
constexpr auto RETURN_HOME_AND_OPEN_GRIPPER_NAME = "JengaReturnHomeAndOpenGripper";

} //namespace Bloom

} //namespace Motions

enum JengaEndStrategy {
  SWAP_TOWERS,
  TRANSITION_TO_IDLE_STATE
};

enum BloomEndStrategy {
  PLACE_AND_RETURN_HOME,
  WAIT_AFTER_TRANSPORT
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMDEFINES_H_ */
