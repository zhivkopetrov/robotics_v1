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
  ErrorCode setTransportStrategy(int32_t strategyId) override;

private:
  struct BloomState {
    bool holdingObject = false;
    bool reachedTransportTargetPose = false;
  };

  //==========START stateful commands===========
  UrscriptCommand generateGraspCommand();

  //used for BloomEndStrategy::PLACE_AND_RETURN_HOME strategy
  UrscriptCommand generatePlaceCommand();

  //used for BloomEndStrategy::WAIT_AFTER_TRANSPORT strategy
  UrscriptCommand generateHandoverCommand();

  UrscriptCommand generateTransportCommand();

  std::vector<UrscriptCommand> 
    generateGracefullyStopPlaceAndReturnHomeStrategy();
  std::vector<UrscriptCommand> 
    generateGracefullyStopWaitAfterTransportStrategy();
  //===========END stateful commands============

  //used for BloomEndStrategy::PLACE_AND_RETURN_HOME strategy
  UrscriptCommand generateRetractAndReturnHomeCommand() const;
  UrscriptCommand generateReturnHomeCommand() const;
  UrscriptCommand generateGripperActuateCommand(GripperActuateType type) const;

  TransportMoveCommands generateTransportMoveCommands(
    Motion::Bloom::TransportStrategy strategy) const;
  TransportMoveCommands generateBasicTransportMoveCommands() const;
  TransportMoveCommands generateFullRotationTransportMoveCommands() const;
  TransportMoveCommands generateTwistTransportMoveCommands() const;

  void loadState();
  void serializeState();

  const BloomMotionSequenceConfig _cfg;
  BloomState _state;
  Motion::Bloom::TransportStrategy _transportStrategy = 
    Motion::Bloom::TransportStrategy::BASIC;
  std::shared_ptr<StateFileHandler> _stateFileHandler;
};

#endif /* UR_CONTROL_BLOOM_BLOOMMOTIONSEQUENCE_H_ */
