#ifndef UR_CONTROL_BLOOM_PARKMOTIONSEQUENCE_H_
#define UR_CONTROL_BLOOM_PARKMOTIONSEQUENCE_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ur_control_common/motion/MotionSequence.h"

//Own components headers
#include "ur_control_bloom/motion/config/ParkMotionSequenceConfig.h"
#include "ur_control_bloom/helpers/StateFileHandler.h"

//Forward declarations

class ParkMotionSequence final : public MotionSequence {
public:
  ParkMotionSequence(
    const ParkMotionSequenceConfig& cfg, const std::string& name, int32_t id, 
    const std::shared_ptr<UrScriptBuilder>& urScriptBuilder,
    const std::shared_ptr<StateFileHandler>& stateFileHandler);

  void start(const UrscriptsBatchDoneCb& cb) override;
  void gracefulStop(const UrscriptsBatchDoneCb& cb) override;
  void recover(const UrscriptsBatchDoneCb& cb) override;
  ErrorCode setTransportStrategy(int32_t strategyId) override;

private:
  UrscriptCommand generateReturnHomeCommand() const;

  const ParkMotionSequenceConfig _cfg;
};

#endif /* UR_CONTROL_BLOOM_PARKMOTIONSEQUENCE_H_ */
