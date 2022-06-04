//Corresponding header
#include "robo_cleaner_gui/helpers/SolutionValidator.h"

//System headers
#include <algorithm>

//Other libraries headers
#include "utils/Log.h"

//Own components headers
#include "robo_cleaner_gui/helpers/config/SolutionValidatorConfig.h"

ErrorCode SolutionValidator::init(
    const SolutionValidatorConfig &cfg,
    const SolutionValidatorOutInterface &outInterface) {
  if (ErrorCode::SUCCESS != initOutInterface(outInterface)) {
    LOGERR("Error, initOutInterface() failed");
    return ErrorCode::FAILURE;
  }

  _validationOptions.targetMapTilesCount = cfg.targetMapTilesCount;
  _reveleadMapTiles.insert(cfg.playerStartLocation);

  return ErrorCode::SUCCESS;
}

void SolutionValidator::fieldMapRevealed() {
  _validationOptions.fieldMapReveleaded = true;
}

ValidationResult SolutionValidator::validateFieldMap(
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

ValidationResult SolutionValidator::handleNormalMove(const FieldPos &fieldPos) {
  ValidationResult result;
  if (_validationOptions.targetMapTilesCount == _reveleadMapTiles.size()) {
    result.success = false;
    return result;
  }

  const auto [_, success] = _reveleadMapTiles.insert(fieldPos);
  result.success = success;
  return result;
}

ErrorCode SolutionValidator::initOutInterface(
    const SolutionValidatorOutInterface &outInterface) {
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


