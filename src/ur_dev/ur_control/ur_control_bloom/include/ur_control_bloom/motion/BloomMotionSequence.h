#ifndef UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCE_H_
#define UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCE_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ur_control_common/motion/MotionSequence.h"

//Own components headers

//Forward declarations

class BloomMotionSequence final : public MotionSequence {
public:
  BloomMotionSequence(
    const std::string& name, int32_t id, UrScriptHeaders&& headers);

  ErrorCode init(const std::any& cfg) override;
  void start(const MotionCommandBatchDoneCb& cb) override;
  void gracefulStop(const MotionCommandBatchDoneCb& cb) override;
  void recover(const MotionCommandBatchDoneCb& cb) override;

private:
  ErrorCode validateUrscriptHeaders() const;
};

#endif /* UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCE_H_ */
