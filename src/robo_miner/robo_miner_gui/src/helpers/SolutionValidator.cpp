//Corresponding header
#include "robo_miner_gui/helpers/SolutionValidator.h"

//C system headers

//C++ system headers
#include <algorithm>

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers
#include "robo_miner_gui/helpers/config/SolutionValidatorConfig.h"

int32_t SolutionValidator::init(
    const SolutionValidatorConfig &cfg,
    const GetFieldDescriptionCb &getFieldDescriptionCb) {
  if (nullptr == getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return FAILURE;
  }
  _getFieldDescriptionCb = getFieldDescriptionCb;

  _longestSequence = cfg.longestSequence;
  const size_t uniquesCount = std::unique(_longestSequence.begin(),
                                  _longestSequence.end())
                              - _longestSequence.begin();
  if (uniquesCount != cfg.longestSequence.size()) {
    LOGERR("Error, provided longestSequence solution contains duplicates");
    return FAILURE;
  }

  std::sort(_longestSequence.begin(), _longestSequence.end());
  _validationOptions.longestSequencePointsValidated.resize(uniquesCount, false);

  return SUCCESS;
}

bool SolutionValidator::validateFieldMap(const std::vector<uint8_t> &rawData,
                                         uint32_t rows, uint32_t cols,
                                         std::string &outError) {
  if (0 == rows) {
    outError = "Invalid arguments. 'rows' args can't be 0";
    return false;
  }

  if (0 == cols) {
    outError = "Invalid arguments. 'cols' args can't be 0";
    return false;
  }

  FieldData data(rows);
  for (uint32_t row = 0; row < rows; ++row) {
    const auto startElemId = row * cols;
    const auto endElemId = startElemId + cols;
    std::copy(rawData.begin() + startElemId, rawData.begin() + endElemId,
        std::back_inserter(data[row]));
  }

  const auto &fieldData = _getFieldDescriptionCb().data;
  if (fieldData != data) {
    outError = "Incorrect FieldMap provided. Try again";
    return false;
  }

  _validationOptions.fieldMapValidated = true;
  return true;
}

bool SolutionValidator::validateLongestSequence(
    CrystalSequence &sequence, std::string &outError) {
  if (!_validationOptions.fieldMapValidated) {
    outError = "Service is locked. FieldMap needs to be validated first";
    return false;
  }

  std::sort(sequence.begin(), sequence.end());

  CrystalSequence diff;
  std::set_difference(_longestSequence.begin(), _longestSequence.end(),
      sequence.begin(), sequence.end(), std::inserter(diff, diff.begin()));
  if (!diff.empty()) {
    outError = "Incorrect sequence provided. Try again";
    return false;
  }

  _validationOptions.longestSequenceValidated = true;
  return true;
}

