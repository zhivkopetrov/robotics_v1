#ifndef UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCE_H_
#define UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCE_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ur_control_common/motion/MotionSequence.h"

//Own components headers
#include "ur_control_bloom/motion/config/JengaMotionSequenceConfig.h"

//Forward declarations

class JengaMotionSequence final : public MotionSequence {
public:
  JengaMotionSequence(
    const JengaMotionSequenceConfig& cfg, const std::string& name, int32_t id);

  void start(const MotionCommandBatchDoneCb& cb) override;
  void gracefulStop(const MotionCommandBatchDoneCb& cb) override;
  void recover(const MotionCommandBatchDoneCb& cb) override;

private:
  void populateUrscriptHeaders();

  const JengaMotionSequenceConfig _cfg;
};

#endif /* UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCE_H_ */
