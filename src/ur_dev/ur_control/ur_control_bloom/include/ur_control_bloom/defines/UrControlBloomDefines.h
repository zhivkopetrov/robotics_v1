#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMDEFINES_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMDEFINES_H_

//System headers

//Other libraries headers

//Own components headers

//Forward declarations

namespace BloomState {

constexpr auto SECTION_NAME = "BloomState";
constexpr auto STATE_ENTRY_NAME = "State";

constexpr auto STATES_COUNT = 7;
  
constexpr auto INIT = "Init";
constexpr auto IDLE = "Idle";
constexpr auto PARK = "Park";
constexpr auto BLOOM = "Bloom";
constexpr auto BLOOM_RECOVERY = "Bloom Recovery";
constexpr auto JENGA = "Jenga";
constexpr auto JENGA_RECOVERY = "Jenga Recovery";

} //namespace BloomState

namespace Motion {

enum MotionId {
  BLOOM_MOTION_ID,
  JENGA_MOTION_ID,
  PARK_ID
};

constexpr auto BLOOM_MOTION_SEQUENCE_NAME = "BloomMotionSequence";
constexpr auto JENGA_MOTION_SEQUENCE_NAME = "JengaMotionSequence";
constexpr auto PARK_MOTION_SEQUENCE_NAME = "ParkMotionSequence";

namespace Bloom {

enum class TransportStrategy {
  BASIC,
  FULL_ROTATION,
  TWIST
};

constexpr auto SECTION_NAME = "BloomMotion";
constexpr auto HOLDING_OBJECT_ENTRY_NAME = "HoldingObject";
constexpr auto REACHED_TRANSPORT_TARGET_POSE_ENTRY_NAME = 
  "ReachedTransportTargetPose";

constexpr auto GRASP_NAME = "BloomGrasp";
constexpr auto TRANSPORT_NAME = "BloomTransport";
constexpr auto TRANSPORT_AND_WAIT_NAME = "BloomTransportAndWait";
constexpr auto PLACE_NAME = "BloomPlace";
constexpr auto RETURN_HOME_NAME = "BloomReturnHome";
constexpr auto RETRACT_AND_RETURN_HOME_NAME = "BloomRetractAndReturnHome";
constexpr auto OPEN_GRIPPER_NAME = "BloomOpenGripper";
constexpr auto CLOSE_GRIPPER_NAME = "BloomCloseGripper";

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

} //namespace Jenga

namespace Park {

constexpr auto PARK_NAME = "Park";

} //namespace Park

} //namespace Motions

namespace Gripper {

constexpr auto ACTIVATE_NAME = "GripperActivate";
constexpr auto OPEN_NAME = "GripperOpen";
constexpr auto CLOSE_NAME = "GripperClose";

} //namespace Gripper

enum CustomActionButtonDefines {
  PARK_IDX,
  JENGA_IDX, 
  BLOOM_RANDOMIZED_IDX, 
  BLOOM_1ST_IDX, 
  BLOOM_2ND_IDX, 
  BLOOM_3RD_IDX, 
  GRACEFUL_STOP_IDX,
  ABORT_MOTION_IDX,
  CUSTOM_ACTION_BUTTONS_COUNT
};

enum JengaEndStrategy {
  SWAP_TOWERS,
  TRANSITION_TO_IDLE_STATE
};

enum BloomEndStrategy {
  PLACE_AND_RETURN_HOME,
  WAIT_AFTER_TRANSPORT
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMDEFINES_H_ */
