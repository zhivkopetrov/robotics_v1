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

  void start(const UrscriptsBatchDoneCb& cb) override;
  void gracefulStop(const UrscriptsBatchDoneCb& cb) override;
  void recover(const UrscriptsBatchDoneCb& cb) override;

private:
  UscriptCommand generateGraspCommand();
  UscriptCommand generateTransportAndPlaceCommand();
  UscriptCommand generateReturnHomeCommand();
  UscriptCommand generateReturnHomeAndOpenGripperCommand();

  void handleSuccessfulPlacement();

  void loadState();
  void serializeState();

  enum class TowerDirection {
    A_TO_B,
    B_TO_A
  };

  struct JengaState {
    int32_t currentObjectIdx = 0;
    int32_t totalObjectsCount = 0;
    TowerDirection towerDirection = TowerDirection::A_TO_B;
    bool holdingObject = false;
  };

  const JengaMotionSequenceConfig _cfg;
  JengaState _state;
  std::shared_ptr<StateFileHandler> _stateFileHandler;
};

#endif /* UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCE_H_ */
