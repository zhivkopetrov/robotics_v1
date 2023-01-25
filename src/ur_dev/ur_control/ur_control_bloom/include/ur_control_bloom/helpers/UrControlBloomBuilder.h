#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMBUILDER_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMBUILDER_H_

//System headers
#include <cstdint>
#include <memory>

//Other libraries headers

//Own components headers
#include "ur_control_bloom/UrControlBloom.h"

//Forward declarations
class Ros2Communicator;

class UrControlBloomBuilder {
public:
  UrControlBloomBuilder() = delete;

  static std::unique_ptr<UrControlBloom> createUrControlBloom(
      const std::unique_ptr<Ros2Communicator>& communicator);
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMBUILDER_H_ */
