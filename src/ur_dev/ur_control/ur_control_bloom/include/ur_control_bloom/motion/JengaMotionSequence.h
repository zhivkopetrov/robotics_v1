#ifndef UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCE_H_
#define UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCE_H_

//System headers
#include <cstdint>

//Other libraries headers
#include "ur_control_common/motion/MotionSequence.h"

//Own components headers
#include "ur_control_bloom/motion/config/JengaMotionSequenceConfig.h"
#include "ur_control_bloom/helpers/StateFileHandler.h"

//Forward declarations

class JengaMotionSequence final : public MotionSequence {
public:
  JengaMotionSequence(
    const JengaMotionSequenceConfig& cfg, const std::string& name, int32_t id,
    const std::shared_ptr<UrScriptBuilder>& urScriptBuilder,
    const std::shared_ptr<StateFileHandler>& stateFileHandler);

  void start(const UscriptsBatchDoneCb& cb) override;
  void gracefulStop(const UscriptsBatchDoneCb& cb) override;
  void recover(const UscriptsBatchDoneCb& cb) override;

private:
  struct JengaState {
    bool holdingObject = false;
  };

  void loadState();
  void serializeState();

  const JengaMotionSequenceConfig _cfg;
  JengaState _state;
  std::shared_ptr<StateFileHandler> _stateFileHandler;
};

#endif /* UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCE_H_ */
