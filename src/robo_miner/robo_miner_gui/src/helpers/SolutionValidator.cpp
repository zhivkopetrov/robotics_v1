//Corresponding header
#include "robo_miner_gui/helpers/SolutionValidator.h"

//C system headers

//C++ system headers
#include <algorithm>

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t SolutionValidator::init(
    const GetFieldDescriptionCb &getFieldDescriptionCb) {
  if (nullptr == getFieldDescriptionCb) {
    LOGERR("Error, nullptr provided for GetFieldDescriptionCb");
    return FAILURE;
  }
  _getFieldDescriptionCb = getFieldDescriptionCb;

  return SUCCESS;
}

bool SolutionValidator::validateSolution(const std::vector<uint8_t> &rawData,
                                         uint32_t rows, uint32_t cols,
                                         std::string &outError) const {
  if (0 == rows) {
    outError = "invalid arguments. rows args can't be 0";
    return false;
  }

  if (0 == cols) {
    outError = "invalid arguments. cols args can't be 0";
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
    outError = "field mismatch";
    return false;
  }

  return true;
}
