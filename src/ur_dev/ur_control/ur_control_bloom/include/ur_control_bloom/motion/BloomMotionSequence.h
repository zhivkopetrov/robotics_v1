#ifndef UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCE_H_
#define UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCE_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ur_control_common/motion/MotionSequence.h"

//Own components headers
#include "ur_control_bloom/motion/config/BloomMotionSequenceConfig.h"

//Forward declarations

class BloomMotionSequence final : public MotionSequence {
public:
  BloomMotionSequence(
    const BloomMotionSequenceConfig& cfg, const std::string& name, int32_t id);

  void start(const MotionCommandBatchDoneCb& cb) override;
  void gracefulStop(const MotionCommandBatchDoneCb& cb) override;
  void recover(const MotionCommandBatchDoneCb& cb) override;

private:
  void populateUrscriptHeaders();

  const BloomMotionSequenceConfig _cfg;
};

#endif /* UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCE_H_ */
