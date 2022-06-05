//Corresponding header
#include "robo_cleaner_gui/helpers/RoboCleanerSolutionValidator.h"

//System headers
#include <algorithm>

//Other libraries headers
#include "robo_common/layout/field/FieldUtils.h"
#include "utils/Log.h"

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

ValidationResult RoboCleanerSolutionValidator::validateFieldMap(
    const std::vector<uint8_t> &rawData, uint32_t rows, uint32_t cols,
    std::string &outError) {
  ValidationResult result;
  if (!_validationOptions.fieldMapReveleaded) {
    outError = "Whole FieldMap is still not revealed";
    result.success = false;
    return result;
  }

  if (_validationOptions.fieldMapValidated) {
    outError = "FieldMap was already validated";
    result.success = false;
    return result;
  }

  if (0 == rows) {
    outError = "Invalid arguments. 'rows' args can't be 0";
    result.success = false;
    return result;
  }

  if (0 == cols) {
    outError = "Invalid arguments. 'cols' args can't be 0";
    result.success = false;
    return result;
  }

  FieldData data(rows);
  for (uint32_t row = 0; row < rows; ++row) {
    const auto startElemId = row * cols;
    const auto endElemId = startElemId + cols;
    std::copy(rawData.begin() + startElemId, rawData.begin() + endElemId,
        std::back_inserter(data[row]));
  }

  const auto &fieldData = _outInterface.getFieldDescriptionCb().data;
  if (fieldData != data) {
    auto &tries = _validationOptions.fieldMapValidationsTriesLeft;
    --tries;
    outError = "Incorrect FieldMap provided. Tries left: ";
    outError.append(std::to_string(tries));
    result.success = false;

    if (0 == tries) {
      result.majorError = true;
    }
    return result;
  }

  _validationOptions.fieldMapValidated = true;
  return result;
}

char RoboCleanerSolutionValidator::handleMoveRequest(MoveType moveType) {
  const RobotState robotState = _outInterface.getRobotStateCb();
  const FieldDescription& fieldDescr = _outInterface.getFieldDescriptionCb();

  //rotate in current pos. Tile is already revealed
  if ((MoveType::ROTATE_LEFT == moveType) ||
      (MoveType::ROTATE_RIGHT == moveType)) {
    return fieldDescr.data[robotState.fieldPos.row][robotState.fieldPos.col];
  }

  //process forward movement
  const FieldPos futurePos =
      FieldUtils::getAdjacentPos(robotState.dir, robotState.fieldPos);

  //don't insert the pos immediately. The move might be canceled
  auto it = _reveleadMapTiles.find(futurePos);
  if (it != _reveleadMapTiles.end()) {
    return fieldDescr.data[futurePos.row][futurePos.col];
  }

  return RoboCommonDefines::UNKNOWN_FIELD_MARKER;
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

void RoboCleanerSolutionValidator::finishMove(const FieldPos& fieldPos) {
  _reveleadMapTiles.insert(fieldPos);
}


