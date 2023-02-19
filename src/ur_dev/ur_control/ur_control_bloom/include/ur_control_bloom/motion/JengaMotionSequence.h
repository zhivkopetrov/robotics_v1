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
  void performRemainingTowerSequence(const UrscriptsBatchDoneCb& cb);

  UrscriptCommand generateGraspCommand(int32_t currObjIdx);
  UrscriptCommand generateTransportAndPlaceCommand(int32_t currObjIdx);
  UrscriptCommand generateReturnHomeCommand();
  UrscriptCommand generateReturnHomeAndOpenGripperCommand();

  void handleSuccessfulPlacement();

  WaypointCartesian computeObjectPose(
    const Point3d& towerCenter, int32_t objectIdx) const;

  void loadState();
  void serializeState();

  enum class TowerDirection {
    A_TO_B,
    B_TO_A
  };

  struct JengaState {
    int32_t currentObjectIdx = 0;
    TowerDirection towerDirection = TowerDirection::A_TO_B;
    bool holdingObject = false;
  };

  const JengaMotionSequenceConfig _cfg;
  JengaState _state;
  std::shared_ptr<StateFileHandler> _stateFileHandler;
};

#endif /* UR_CONTROL_BLOOM_JENGAMOTIONSEQUENCE_H_ */
