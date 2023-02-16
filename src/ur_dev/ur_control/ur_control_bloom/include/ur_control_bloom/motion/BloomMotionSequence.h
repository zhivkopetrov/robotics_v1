#ifndef UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCE_H_
#define UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCE_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ur_control_common/motion/MotionSequence.h"

//Own components headers
#include "ur_control_bloom/motion/config/BloomMotionSequenceConfig.h"
#include "ur_control_bloom/helpers/StateFileHandler.h"

//Forward declarations

class BloomMotionSequence final : public MotionSequence {
public:
  BloomMotionSequence(
    const BloomMotionSequenceConfig& cfg, const std::string& name, int32_t id, 
    const std::shared_ptr<UrScriptBuilder>& urScriptBuilder,
    const std::shared_ptr<StateFileHandler>& stateFileHandler);

  void start(const UscriptsBatchDoneCb& cb) override;
  void gracefulStop(const UscriptsBatchDoneCb& cb) override;
  void recover(const UscriptsBatchDoneCb& cb) override;

  void abort(const UscriptsBatchDoneCb& cb);

private:
  void serializeState(MotionAction action);
  
  const BloomMotionSequenceConfig _cfg;
  std::shared_ptr<StateFileHandler> _stateFileHandler;
};

#endif /* UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCE_H_ */
