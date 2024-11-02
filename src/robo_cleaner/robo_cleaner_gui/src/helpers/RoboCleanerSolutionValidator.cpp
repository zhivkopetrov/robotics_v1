//Corresponding header
#include "robo_cleaner_gui/helpers/RoboCleanerSolutionValidator.h"

//System headers
#include <algorithm>

//Other libraries headers
#include "robo_cleaner_common/defines/RoboCleanerDefines.h"
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/log/Log.h"

//Own components headers
#include "robo_cleaner_gui/helpers/config/RoboCleanerSolutionValidatorConfig.h"

ErrorCode RoboCleanerSolutionValidator::init(
    const RoboCleanerSolutionValidatorConfig &cfg,
    const RoboCleanerSolutionValidatorOutInterface &outInterface) {
  if (ErrorCode::SUCCESS != initOutInterface(outInterface)) {
    LOGERR("Error, initOutInterface() failed");
    return ErrorCode::FAILURE;
  }

  _reveleadMapTiles.insert(cfg.playerStartLocation);

  return ErrorCode::SUCCESS;
}

void RoboCleanerSolutionValidator::fieldMapRevealed() {
  _validationOptions.fieldMapReveleaded = true;
}

void RoboCleanerSolutionValidator::fieldMapCleaned() {
  _validationOptions.fieldMapCleaned = true;
}

ValidationResult RoboCleanerSolutionValidator::queryInitialRobotPos(
    InitialRobotState &outRobotState, std::string &outError) {
  ValidationResult result;

  if (_validationOptions.initialRobotStateRequested) {
    outError = "Initial Robot State could be queried only once";
    result.success = false;
    result.majorError = true;
  }
  _validationOptions.initialRobotStateRequested = true;

  const RobotState state = _outInterface.getRobotStateCb();
  const auto& fieldDescr = _outInterface.getFieldDescriptionCb();
  outRobotState.robotTile =
      fieldDescr.data[state.fieldPos.row][state.fieldPos.col];
  outRobotState.robotDir = state.dir;

  return result;
}

char RoboCleanerSolutionValidator::getApproachingTileMarker(
    MoveType moveType) const {
  const RobotState robotState = _outInterface.getRobotStateCb();
  const FieldDescription &fieldDescr = _outInterface.getFieldDescriptionCb();

  //rotate in current pos. Tile is already revealed
  if ( (MoveType::ROTATE_LEFT == moveType) || (MoveType::ROTATE_RIGHT
      == moveType)) {
    return fieldDescr.data[robotState.fieldPos.row][robotState.fieldPos.col];
  }

  //process forward movement
  const FieldPos futurePos = FieldUtils::getAdjacentPos(robotState.dir,
      robotState.fieldPos);

  //don't insert the pos immediately. The move might be canceled before robot
  //has reached it's destination
  auto it = _reveleadMapTiles.find(futurePos);
  const bool tileAlreadyRevealed = it != _reveleadMapTiles.end();
  if (tileAlreadyRevealed) {
    if (!FieldUtils::isInsideField(futurePos, fieldDescr)) {
      return RoboCommonDefines::FIELD_OUT_OF_BOUND_MARKER;
    }

    return fieldDescr.data[futurePos.row][futurePos.col];
  }

  return RoboCommonDefines::UNKNOWN_FIELD_MARKER;
}

MoveValidation RoboCleanerSolutionValidator::finishMove(
    const RobotState &state, MoveOutcome outcome, MoveType moveType) {
  MoveValidation result;
  const FieldDescription& fieldDescr = _outInterface.getFieldDescriptionCb();
  result.processedMarker =
      fieldDescr.data[state.fieldPos.row][state.fieldPos.col];

  //a previous move should have already populated the required data
  if (MoveType::FORWARD != moveType) {
    return result;
  }

  //mark the collision obstacle as visited
  if (MoveOutcome::COLLISION == outcome) {
    const FieldPos futurePos =
        FieldUtils::getAdjacentPos(state.dir, state.fieldPos);
    _reveleadMapTiles.insert(futurePos);
    return result;
  }

  if (isRubbishMarker(result.processedMarker)) {
    const int32_t rubbishCounter = getRubbishCounter(result.processedMarker);
    if (0 < rubbishCounter) {
      //clean tile (subtract field counter value)
      result.processedMarker--;
      result.tileCleaned = true;
    }
  }

  const auto [_, success] = _reveleadMapTiles.insert(state.fieldPos);
  result.tileRevealed = success;

  result.reachedEndGameCondition = (
      (RoboCleanerDefines::CHARGING_STATION == result.processedMarker) &&
      _validationOptions.fieldMapCleaned &&
      _validationOptions.fieldMapReveleaded);

  return result;
}

ErrorCode RoboCleanerSolutionValidator::initOutInterface(
    const RoboCleanerSolutionValidatorOutInterface &outInterface) {
  _outInterface = outInterface;
  if (nullptr == _outInterface.getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return ErrorCode::FAILURE;
  }

  if (nullptr == _outInterface.getRobotStateCb) {
    LOGERR("Error, nullptr provided for GetRobotStateCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

void RoboCleanerSolutionValidator::increaseTotalRobotMovesCounter(
    int32_t movesCount) {
  _totalRobotMoves += movesCount;
}

int32_t RoboCleanerSolutionValidator::getTotalRobotMovesCounter() const {
  return _totalRobotMoves;
}

bool RoboCleanerSolutionValidator::isRobotAtChargingStation() const {
  const FieldPos robotPos = _outInterface.getRobotStateCb().fieldPos;
  const FieldData& fieldData = _outInterface.getFieldDescriptionCb().data;
  const char currRobotFieldMarker = fieldData[robotPos.row][robotPos.col];

  return RoboCleanerDefines::CHARGING_STATION == currRobotFieldMarker;
}

