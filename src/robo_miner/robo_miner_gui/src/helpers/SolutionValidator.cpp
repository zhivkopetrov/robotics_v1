//Corresponding header
#include "robo_miner_gui/helpers/SolutionValidator.h"

//System headers
#include <algorithm>

//Other libraries headers
#include "utils/log/Log.h"

//Own components headers
#include "robo_miner_gui/helpers/config/SolutionValidatorConfig.h"

namespace {
constexpr bool UNMINED_CRYSTAL = false;
constexpr bool MINED_CRYSTAL = true;
}

ErrorCode SolutionValidator::init(
    const SolutionValidatorConfig &cfg,
    const SolutionValidatorOutInterface &outInterface) {
  if (ErrorCode::SUCCESS != initOutInterface(outInterface)) {
    LOGERR("Error, initOutInterface() failed");
    return ErrorCode::FAILURE;
  }

  _validationOptions.targetMapTilesCount = cfg.targetMapTilesCount;
  _longestSequence = cfg.longestSequence;
  const size_t uniquesCount = std::unique(_longestSequence.begin(),
                                  _longestSequence.end())
                              - _longestSequence.begin();
  if (uniquesCount != cfg.longestSequence.size()) {
    LOGERR("Error, provided longestSequence solution contains duplicates");
    return ErrorCode::FAILURE;
  }

  std::sort(_longestSequence.begin(), _longestSequence.end());
  _validationOptions.longestSequenceValidationPoints.resize(uniquesCount,
      UNMINED_CRYSTAL);

  _reveleadMapTiles.insert(cfg.playerStartLocation);

  return ErrorCode::SUCCESS;
}

void SolutionValidator::fieldMapRevealed() {
  _validationOptions.fieldMapReveleaded = true;
}

ValidationResult SolutionValidator::queryInitialRobotPos(
    InitialRobotPos& outRobotPos, std::string &outError) {
  ValidationResult result;

  if (_validationOptions.initialRobotPosRequested) {
    outError = "Initial Robot Position could be queried only once";
    result.success = false;
    result.majorError = true;
  }
  _validationOptions.initialRobotPosRequested = true;
  outRobotPos.surroundingTiles = _outInterface.getPlayerSurroundingTilesCb();

  const RobotState state = _outInterface.getRobotStateCb();
  const auto& fieldDescr = _outInterface.getFieldDescriptionCb();
  outRobotPos.robotTile =
      fieldDescr.data[state.fieldPos.row][state.fieldPos.col];
  outRobotPos.robotDir = state.dir;

  return result;
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

ValidationResult SolutionValidator::validateLongestSequence(
    CrystalSequence &sequence, std::string &outError) {
  ValidationResult result;
  if (!_validationOptions.fieldMapValidated) {
    outError = "Service is locked. FieldMap needs to be validated first";
    result.success = false;
    return result;
  }

  if (_validationOptions.longestSequenceValidated) {
    outError = "Longest sequence was already validated";
    result.success = false;
    return result;
  }

  std::sort(sequence.begin(), sequence.end());

  CrystalSequence diff;
  std::set_difference(_longestSequence.begin(), _longestSequence.end(),
      sequence.begin(), sequence.end(), std::inserter(diff, diff.begin()));
  if (!diff.empty()) {
    auto &tries = _validationOptions.longestSequenceValidationsTriesLeft;
    --tries;
    outError = "Incorrect longest sequence provided. Tries left: ";
    outError.append(std::to_string(tries));
    result.success = false;

    if (0 == tries) {
      result.majorError = true;
    }
    return result;
  }

  _validationOptions.longestSequenceValidated = true;
  return result;
}

ValidationResult SolutionValidator::handleNormalMove(const FieldPos &fieldPos) {
  ValidationResult result;
  if (_validationOptions.miningActivated) {
    LOGERR("Error, normalMoves should not be executed when mining "
           "has been started");
    result.success = false;
    return result;
  }

  if (_validationOptions.targetMapTilesCount == _reveleadMapTiles.size()) {
    result.success = false;
    return result;
  }

  const auto [_, success] = _reveleadMapTiles.insert(fieldPos);
  result.success = success;
  return result;
}

ValidationResult SolutionValidator::handleMiningMove(const FieldPos &fieldPos) {
  ValidationResult result;
  if (!_validationOptions.miningActivated) {
    result.success = false;
    return result;
  }

  size_t longestSequenceIdx{};
  const auto success = validateMiningPos(fieldPos, longestSequenceIdx);
  if (!success) {
    result.success = false;
    result.majorError = true;
    return result;
  }

  auto &validationPoints = _validationOptions.longestSequenceValidationPoints;
  if (MINED_CRYSTAL == validationPoints[longestSequenceIdx]) {
    //found a point, which is already marked as 'mined'
    result.success = false;
    return result;
  }
  validationPoints[longestSequenceIdx] = MINED_CRYSTAL;

  return result;
}

ValidationResult SolutionValidator::validateActivateMining(
    std::string &outError) {
  ValidationResult result;
  if (!_validationOptions.longestSequenceValidated) {
    outError =
        "Service is locked. Longest sequence needs to be validated first";
    result.success = false;
    return result;
  }

  if (_validationOptions.miningActivated) {
    outError = "Mining was already activated";
    result.success = false;
    return result;
  }

  const auto robotFieldPos = _outInterface.getRobotStateCb().fieldPos;
  size_t longestSequenceIdx{};
  const bool success = validateMiningPos(robotFieldPos, longestSequenceIdx);
  if (!success) {
    outError = "Initial mining position row, col [";
    outError.append(std::to_string(robotFieldPos.row)).append(",").append(
        std::to_string(robotFieldPos.col)).
        append("] is outside of the longest sequence boundaries");
    result.success = false;
    result.majorError = true;
    return result;
  }

  _validationOptions.longestSequenceValidationPoints[longestSequenceIdx] =
      MINED_CRYSTAL;
  _validationOptions.miningActivated = true;
  return result;
}

bool SolutionValidator::isMiningActive() const {
  return _validationOptions.miningActivated;
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

  if (nullptr == _outInterface.getPlayerSurroundingTilesCb) {
    LOGERR("Error, nullptr provided for GetPlayerSurroundingTilesCb");
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

bool SolutionValidator::validateMiningPos(
    const FieldPos &fieldPos, size_t &foundLongestSequenceIdx) const {
  const auto it = std::find(_longestSequence.begin(), _longestSequence.end(),
      fieldPos);
  if (it == _longestSequence.end()) {
    LOGR("Mining FieldPos row, col [%d,%d] outside of longest sequence "
         "boundaries", fieldPos.row, fieldPos.col);
    return false;
  }

  foundLongestSequenceIdx = it - _longestSequence.begin();
  return true;
}

