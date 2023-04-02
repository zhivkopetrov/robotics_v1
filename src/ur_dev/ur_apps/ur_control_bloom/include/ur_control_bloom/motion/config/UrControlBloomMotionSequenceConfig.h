#ifndef UR_CONTROL_BLOOM_URCONTROLBLOOMMOTIONSEQUENCECONFIG_H_
#define UR_CONTROL_BLOOM_URCONTROLBLOOMMOTIONSEQUENCECONFIG_H_

//System headers

//Other libraries headers

//Own components headers
#include "ur_control_bloom/motion/config/BloomMotionSequenceConfig.h"
#include "ur_control_bloom/motion/config/JengaMotionSequenceConfig.h"
#include "ur_control_bloom/motion/config/ParkMotionSequenceConfig.h"

//Forward declarations

struct UrControlBloomMotionSequenceConfig {
  BloomMotionSequenceConfig bloomMotionSequenceCfg;
  JengaMotionSequenceConfig jengaMotionSequenceCfg;
  ParkMotionSequenceConfig parkMotionSequenceCfg;
};

#endif /* UR_CONTROL_BLOOM_URCONTROLBLOOMMOTIONSEQUENCECONFIG_H_ */
