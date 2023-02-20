//Corresponding header
#include "ur_control_bloom/motion/JengaMotionSequence.h"

//System headers

//Other libraries headers
#include "urscript_common/motion/MotionUtils.h"
#include "utils/Log.h"

//Own components headers
#include "ur_control_bloom/defines/UrControlBloomDefines.h"

namespace {
constexpr auto TOWER_DIR_A_TO_B_STR = "A_TO_B";
constexpr auto TOWER_DIR_B_TO_A_STR = "B_TO_A";
}

JengaMotionSequence::JengaMotionSequence(
  const JengaMotionSequenceConfig& cfg, const std::string& name, int32_t id, 
  const std::shared_ptr<UrScriptBuilder>& urScriptBuilder,
  const std::shared_ptr<StateFileHandler>& stateFileHandler) 
  : MotionSequence(name, id, urScriptBuilder), _cfg(cfg),
  _stateFileHandler(stateFileHandler) {
  loadState();
}

void JengaMotionSequence::start(const UrscriptsBatchDoneCb& cb) {
  auto commands = generateFullPickAndPlaceCommandCycle();
  if (JengaEndStrategy::TRANSITION_TO_IDLE_STATE == _cfg.endStrategy) {
    commands.push_back(generateReturnHomeCommand());
    dispatchUscriptsAsyncCb(commands, cb);
    return;
  }

  //JengaEndStrategy::SWAP_TOWERS
  //after jenga tower is completed, same method ::start() will be called,
  //thus the vice-versa construction will begin
  //this will continue indefinitely or until gracefully_stopped/aborted 
  const UrscriptsBatchDoneCb callSameMethodCb = [this, cb](){
    start(cb);
  };
  dispatchUscriptsAsyncCb(commands, callSameMethodCb);
}

void JengaMotionSequence::gracefulStop(const UrscriptsBatchDoneCb& cb) {
  //for now the graceful_stop and recover implementations are identical
  recover(cb);
}

void JengaMotionSequence::recover(const UrscriptsBatchDoneCb& cb) {
  std::vector<UrscriptCommand> commands { generateReturnHomeCommand() };
  if (_state.holdingObject) {
    const Point3d& towerCenterPos = 
      TowerDirection::A_TO_B == _state.towerDirection ?
        _cfg.baseCenterBCartesian.pos : _cfg.baseCenterBCartesian.pos;
    const WaypointCartesian placeWaypoint = 
      computeObjectPose(towerCenterPos, _state.currentObjectIdx);

    commands.push_back(
      generateTransportAndPlaceCommand(_cfg.homeCartesian, placeWaypoint));
  }
  commands.push_back(generateReturnHomeAndOpenGripperCommand());

  dispatchUscriptsAsyncCb(commands, cb);
}

UrscriptCommand JengaMotionSequence::generateGraspCommand(
    const WaypointCartesian& currPose, const WaypointCartesian& graspPose) {
  const double blendingRadius = computeSafeBlendingRadius(
    currPose.pos, _cfg.graspApproachCartesian.pos, graspPose.pos);

  auto graspApproachCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.graspApproachCartesian, 
      _cfg.pickAndPlaceVel, _cfg.pickAndPlaceAcc, blendingRadius);
  auto graspCommand = std::make_unique<MoveLinearCommand>(
    graspPose, _cfg.pickAndPlaceVel, _cfg.pickAndPlaceAcc);
  auto closeGripperCommand = 
    std::make_unique<GripperActuateCommand>(GripperActuateType::CLOSE);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(graspApproachCommand))
              .addCommand(std::move(graspCommand))
              .addCommand(std::move(closeGripperCommand));
  const UrScriptPayload cmdPayload = 
    constructUrScript(Motion::Jenga::GRASP_NAME, cmdContainer);

  const UrscriptDoneCb doneCb = [this](){
    _state.holdingObject = true;
    serializeState();
  };
  return { cmdPayload, doneCb };
}

UrscriptCommand JengaMotionSequence::generateTransportAndPlaceCommand(
  const WaypointCartesian& currPose, const WaypointCartesian& placePose) {
  const double blendingRadius = computeSafeBlendingRadius(
    currPose.pos, _cfg.graspApproachCartesian.pos, placePose.pos);

  auto transportApproachCommand = 
    std::make_unique<MoveLinearCommand>(_cfg.graspApproachCartesian, 
      _cfg.pickAndPlaceVel, _cfg.pickAndPlaceAcc, blendingRadius);
  auto placeCommand = std::make_unique<MoveLinearCommand>(
    placePose, _cfg.pickAndPlaceVel, _cfg.pickAndPlaceAcc);
  auto openGripperCommand = 
    std::make_unique<GripperPreciseActuateCommand>(_cfg.gripperOpening);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(transportApproachCommand))
              .addCommand(std::move(placeCommand))
              .addCommand(std::move(openGripperCommand));
  const UrScriptPayload cmdPayload = constructUrScript(
    Motion::Jenga::TRANSPORT_AND_PLACE_NAME, cmdContainer);

  const UrscriptDoneCb doneCb = [this](){
    handleSuccessfulPlacement();
    serializeState();
  };
  return { cmdPayload, doneCb };
}

UrscriptCommand JengaMotionSequence::generateReturnHomeCommand() {
  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(returnHomeCommand));
  const UrScriptPayload cmdPayload = 
    constructUrScript(Motion::Jenga::RETURN_HOME_NAME, cmdContainer);

  return { cmdPayload };
}

UrscriptCommand JengaMotionSequence::generateReturnHomeAndOpenGripperCommand() {
  auto returnHomeCommand = std::make_unique<MoveJointCommand>(_cfg.homeJoint);
  auto openGripperCommand = std::make_unique<GripperActuateCommand>(
    GripperActuateType::OPEN, GripperCommandPolicy::NON_BLOCKING);

  UrScriptCommandContainer cmdContainer;
  cmdContainer.addCommand(std::move(openGripperCommand))
              .addCommand(std::move(returnHomeCommand));
  const UrScriptPayload cmdPayload = constructUrScript(
    Motion::Jenga::RETURN_HOME_AND_OPEN_GRIPPER_NAME, cmdContainer);

  return { cmdPayload };
}

std::vector<UrscriptCommand> 
JengaMotionSequence::generateFullPickAndPlaceCommandCycle() {
  std::vector<UrscriptCommand> commands;
  //reserve enough memory for all urscripts
  constexpr int32_t urscriptsPerBlock = 2;
  constexpr int32_t returnHomeUrscripts = 1;
  const int32_t blocksLeftFromCurrTower = 
    _cfg.totalObjectsPerTower - _state.currentObjectIdx;
  commands.reserve(
    (blocksLeftFromCurrTower * urscriptsPerBlock) + returnHomeUrscripts);

  const auto [graspTowerCenterPos, placeTowerCenterPos] = [this](){
    return TowerDirection::A_TO_B == _state.towerDirection ?
      std::make_pair(
        _cfg.baseCenterACartesian.pos, _cfg.baseCenterBCartesian.pos) :
      std::make_pair(
        _cfg.baseCenterBCartesian.pos, _cfg.baseCenterACartesian.pos);
  }();

  WaypointCartesian graspWaypoint;
  WaypointCartesian placeWaypoint;
  for (int32_t objIdx = _state.currentObjectIdx; 
       objIdx < _cfg.totalObjectsPerTower; ++objIdx) {
    //the indexes for grasping/placing should be mirrored
    graspWaypoint = computeObjectPose(
      graspTowerCenterPos, _cfg.totalObjectsPerTower - objIdx - 1);
    placeWaypoint = computeObjectPose(placeTowerCenterPos, objIdx);

    commands.push_back(generateGraspCommand(graspWaypoint, placeWaypoint));
    commands.push_back(
      generateTransportAndPlaceCommand(placeWaypoint, graspWaypoint));
  }

  return commands;
}

void JengaMotionSequence::handleSuccessfulPlacement() {
  std::string towerStr = TowerDirection::A_TO_B == _state.towerDirection ? 
    TOWER_DIR_A_TO_B_STR : TOWER_DIR_B_TO_A_STR;
  LOGG("Jenga successful placement for Block[%d], Floor[%d], TowerDir: [%s]", 
       _state.currentObjectIdx, _state.currentObjectIdx / 2, towerStr.c_str());

  _state.holdingObject = false;
  ++_state.currentObjectIdx;
  //swap towers
  if (_state.currentObjectIdx >= _cfg.totalObjectsPerTower) {
    _state.currentObjectIdx = 0;
    _state.towerDirection = TowerDirection::A_TO_B == _state.towerDirection ?
      TowerDirection::B_TO_A : TowerDirection::A_TO_B;

    towerStr = TowerDirection::A_TO_B == _state.towerDirection ? 
      TOWER_DIR_A_TO_B_STR : TOWER_DIR_B_TO_A_STR;
    LOGM("Swapping towers. New TowerDir: [%s]", towerStr.c_str());
  }
}

WaypointCartesian JengaMotionSequence::computeObjectPose(
  const Point3d& towerCenter, int32_t objectIdx) const {
  //block position have a repetative nature
  //odd index floors has +- depth, zero orientation
  //even index floors has +- depth, 90 deg rotated orientation
  constexpr int32_t blocksPerRepeat = 4;
  constexpr int32_t oddFloorFirstBlockIdx = 0;
  constexpr int32_t oddFloorSecondBlockIdx = 1;
  constexpr int32_t evenFloorFirstBlockIdx = 2;
  constexpr int32_t evenFloorSecondBlockIdx = 3;
  const int32_t floorIdx = objectIdx / 2;

  Point3d objectPos = towerCenter;
  objectPos.z += floorIdx * _cfg.blockDimensions.height;
  AngleAxis objectOrientation;
  switch (objectIdx % blocksPerRepeat)
  {
  case oddFloorFirstBlockIdx:
    objectPos.x += _cfg.blockDimensions.depth;
    objectOrientation = _cfg.zeroOrientation;
    break;

  case oddFloorSecondBlockIdx:
    objectPos.x -= _cfg.blockDimensions.depth;
    objectOrientation = _cfg.zeroOrientation;
    break;

  case evenFloorFirstBlockIdx:
    objectPos.y += _cfg.blockDimensions.depth;
    objectOrientation = _cfg.ninetyOrientation;
    break;

  case evenFloorSecondBlockIdx:
    objectPos.y -= _cfg.blockDimensions.depth;
    objectOrientation = _cfg.ninetyOrientation;
    break;
  }

  return WaypointCartesian(objectPos, objectOrientation);
}

void JengaMotionSequence::loadState() {
  _state.holdingObject = [this](){
    std::string strValue;
    const ErrorCode errCode = _stateFileHandler->getEntry(
      Motion::Jenga::SECTION_NAME, Motion::Jenga::HOLDING_OBJECT_ENTRY_NAME, 
      strValue);
    if (ErrorCode::SUCCESS != errCode) {
      LOGERR("Error trying to getEntry(): [%s] for section: [%s]. "
            "Defaulting to [%s]", Motion::Jenga::HOLDING_OBJECT_ENTRY_NAME, 
            Motion::Jenga::SECTION_NAME, BOOL_FALSE_VALUE_STR);
      return false;
    }

    if (BOOL_TRUE_VALUE_STR == strValue) {
      return true;
    }
    return false;
  }();

  _state.towerDirection = [this](){
    constexpr auto defaultDir = TowerDirection::A_TO_B;
    std::string strValue;
    const ErrorCode errCode = _stateFileHandler->getEntry(
      Motion::Jenga::SECTION_NAME, Motion::Jenga::DIRECTION_ENTRY_NAME, 
      strValue);
    if (ErrorCode::SUCCESS != errCode) {
      LOGERR("Error trying to getEntry(): [%s] for section: [%s]. "
             "Defaulting to [%s]", Motion::Jenga::DIRECTION_ENTRY_NAME, 
             Motion::Jenga::SECTION_NAME, TOWER_DIR_A_TO_B_STR);
      return defaultDir;
    }

    if (TOWER_DIR_A_TO_B_STR == strValue) {
      return TowerDirection::A_TO_B;
    }
    return TowerDirection::B_TO_A;
  }();

  _state.currentObjectIdx = [this](){
    constexpr auto defaultCurrObjIdx = 0;
    std::string strValue;
    const ErrorCode errCode = _stateFileHandler->getEntry(
      Motion::Jenga::SECTION_NAME, Motion::Jenga::CURRENT_OBJECT_IDX_ENTRY_NAME, 
      strValue);
    if (ErrorCode::SUCCESS != errCode) {
      LOGERR("Error trying to getEntry(): [%s] for section: [%s]. "
             "Defaulting to [%d]", Motion::Jenga::CURRENT_OBJECT_IDX_ENTRY_NAME, 
             Motion::Jenga::SECTION_NAME, defaultCurrObjIdx);
      return defaultCurrObjIdx;
    }

    try {
      return std::stoi(strValue);
    } catch (const std::exception &e) {
      LOGERR("%s", e.what());
      return defaultCurrObjIdx;
    }
  }();
}

void JengaMotionSequence::serializeState() {
  const std::string holdingObjStr = _state.holdingObject ? 
    BOOL_TRUE_VALUE_STR : BOOL_FALSE_VALUE_STR;
  ErrorCode errCode = _stateFileHandler->updateEntry(
    Motion::Jenga::SECTION_NAME, Motion::Jenga::HOLDING_OBJECT_ENTRY_NAME, 
    holdingObjStr);
  if (ErrorCode::SUCCESS != errCode) {
    LOGERR("Error trying to serialize JengaMotionSequenceState");
  }

  errCode = _stateFileHandler->updateEntry(
    Motion::Jenga::SECTION_NAME, Motion::Jenga::CURRENT_OBJECT_IDX_ENTRY_NAME, 
    std::to_string(_state.currentObjectIdx));
  if (ErrorCode::SUCCESS != errCode) {
    LOGERR("Error trying to serialize JengaMotionSequenceState");
  }

  const std::string directionStr = 
    TowerDirection::A_TO_B == _state.towerDirection ? 
      TOWER_DIR_A_TO_B_STR : TOWER_DIR_B_TO_A_STR;
  errCode = _stateFileHandler->updateEntry(
    Motion::Jenga::SECTION_NAME, Motion::Jenga::DIRECTION_ENTRY_NAME, 
    directionStr);
  if (ErrorCode::SUCCESS != errCode) {
    LOGERR("Error trying to serialize JengaMotionSequenceState");
  }
}
