//Corresponding header
#include "robo_miner_gui/helpers/algorithms/FloodFill.h"

//C system headers

//C++ system headers
#include <stack>
#include <array>

//Other libraries headers

//Own components headers

namespace {

constexpr auto PROCESSED_MARKER = '0';

bool isValidMove(char searchMarker, const FieldData &data,
                 const FieldPos &location) {
  if (0 > location.row) {
    return false;
  }

  if (static_cast<int32_t>(data.size()) <= location.row) {
    return false;
  }

  if (0 > location.col) {
    return false;
  }

  if (!data[0].empty() &&
      static_cast<int32_t>(data[0].size()) <= location.col) {
    return false;
  }

  if (searchMarker != data[location.row][location.col]) {
    return false;
  }

  return true;
}

void DFSSearch(char searchMarker, FieldData &data,
               std::stack<FieldPos> &dataPath,
               std::vector<FieldPos> &outLongestSequence) {
  while (!dataPath.empty()) {
    const auto currLocation = dataPath.top();
    dataPath.pop();

    constexpr auto arrSize = 4;
    const std::array<FieldPos, arrSize> dirs = { FieldPos { currLocation.row,
        currLocation.col - 1 }, FieldPos { currLocation.row - 1,
        currLocation.col }, FieldPos { currLocation.row, currLocation.col + 1 },
        FieldPos { currLocation.row + 1, currLocation.col } };

    for (const auto &dir : dirs) {
      if (isValidMove(searchMarker, data, dir)) {
        dataPath.push(dir);
        outLongestSequence.push_back(dir);
        data[dir.row][dir.col] = PROCESSED_MARKER;
      }
    }
  }
}

std::vector<FieldPos> findLocalSequence(
    const std::vector<char> &nonCrystalMarkers, int32_t currRow,
    int32_t currCol, FieldData &data, std::stack<FieldPos> &dataPath) {
  std::vector<FieldPos> sequence;
  const char currMarker = data[currRow][currCol];

  if (PROCESSED_MARKER == currMarker) {
    return sequence;
  }

  for (const auto marker : nonCrystalMarkers) {
    if (marker == currMarker) {
      return sequence;
    }
  }

  const FieldPos fieldPos { currRow, currCol };
  dataPath.push(fieldPos);
  sequence.push_back(fieldPos);
  data[currRow][currCol] = PROCESSED_MARKER;

  DFSSearch(currMarker, data, dataPath, sequence);
  return sequence;
}

} //end anonymous namespace

std::vector<FieldPos> FloodFill::findLongestCrystalSequence(
    const FieldData &data, const std::vector<char> &nonCrystalMarkers) {
  auto localData = data; //will be modified
  const int32_t dataRows = static_cast<int32_t>(data.size());
  const int32_t dataCols = static_cast<int32_t>(data[0].size());
  std::stack<FieldPos> dataPath;

  std::vector<FieldPos> longestCrystalSequence;
  std::vector<FieldPos> currCrystalSequence;

  for (int32_t row = 0; row < dataRows; ++row) {
    for (int32_t col = 0; col < dataCols; ++col) {
      currCrystalSequence = findLocalSequence(nonCrystalMarkers, row, col,
          localData, dataPath);

      if (currCrystalSequence.size() > longestCrystalSequence.size()) {
        longestCrystalSequence = currCrystalSequence;
      }
    }
  }

  return longestCrystalSequence;
}

std::vector<FieldPos> FloodFill::findLocalCrystalSequence(
    const FieldData &data, const std::vector<char> &nonCrystalMarkers,
    const FieldPos &fieldPos) {
  auto localData = data; //will be modified
  std::stack<FieldPos> dataPath;

  return findLocalSequence(nonCrystalMarkers, fieldPos.row, fieldPos.col,
      localData, dataPath);
}

