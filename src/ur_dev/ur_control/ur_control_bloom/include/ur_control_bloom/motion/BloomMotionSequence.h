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

using TransportMoveCommands = std::vector<std::unique_ptr<MoveCommandBase>>;

class BloomMotionSequence final : public MotionSequence {
public:
  BloomMotionSequence(
    const BloomMotionSequenceConfig& cfg, const std::string& name, int32_t id, 
    const std::shared_ptr<UrScriptBuilder>& urScriptBuilder,
    const std::shared_ptr<StateFileHandler>& stateFileHandler);

  void start(const UrscriptsBatchDoneCb& cb) override;
  void gracefulStop(const UrscriptsBatchDoneCb& cb) override;
  void recover(const UrscriptsBatchDoneCb& cb) override;

private:
  enum class TransportStrategy {
    BASIC,
    FULL_ROTATION,
    TWIST
  };

  struct BloomState {
    bool holdingObject = false;
  };

  UrscriptCommand generateGraspCommand();
  UrscriptCommand generateTransportAndPlaceCommand();
  UrscriptCommand generateRetractAndReturnHomeCommand();
  UrscriptCommand generateReturnHomeCommand();
  UrscriptCommand generateReturnHomeAndOpenGripperCommand();

  TransportMoveCommands generateTransportMoveCommands(
    TransportStrategy strategy) const;
  TransportMoveCommands generateBasicTransportMoveCommands() const;
  TransportMoveCommands generateFullRotationTransportMoveCommands() const;
  TransportMoveCommands generateTwistTransportMoveCommands() const;

  void loadState();
  void serializeState();

  const BloomMotionSequenceConfig _cfg;
  BloomState _state;
  std::shared_ptr<StateFileHandler> _stateFileHandler;
};

#endif /* UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCE_H_ */
