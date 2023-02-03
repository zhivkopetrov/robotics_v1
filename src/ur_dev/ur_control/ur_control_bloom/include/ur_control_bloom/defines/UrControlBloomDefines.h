#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMDEFINES_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMDEFINES_H_

//System headers

//Other libraries headers

//Own components headers

//Forward declarations

namespace BloomState {

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

constexpr auto GRASP_NAME = "BloomGrasp";
constexpr auto TRANSPORT_AND_PLACE_NAME = "BloomTransportAndPlace";
constexpr auto RETURN_HOME_NAME = "BloomReturnHome";

} //namespace Bloom

} //namespace Motions

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMDEFINES_H_ */
