#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMTOPICS_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMTOPICS_H_

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

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMTOPICS_H_ */
