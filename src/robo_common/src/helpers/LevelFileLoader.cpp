//Corresponding header
#include "robo_common/helpers/LevelFileLoader.h"

//System headers
#include <fstream>

//Other libraries headers
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/data_type/EnumClassUtils.h"
#include "utils/file_system/FileSystemUtils.h"
#include "utils/debug/StrError.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto FIELD_MAP_FILE_NAME = "field_map.txt";
constexpr auto MINER_LONGEST_SOLUTION_FILE_NAME = "solution.txt";
}

LevelData LevelFileLoader::readLevelData(
    const std::string &projectInstallPrefix, int32_t levelId) {
  LevelData levelData;

  std::string filePath;
  if (ErrorCode::SUCCESS != readLevelFolder(projectInstallPrefix, levelId,
          filePath)) {
    LOGERR("Error, readLevelFolder() failed");
    return {};
  }

  filePath.append("/").append(FIELD_MAP_FILE_NAME);
  std::ifstream ifstream(filePath);
  if (!ifstream) {
    LOGERR("Error, opening file: [%s]. Reason: %s", filePath.c_str(),
        strError().c_str());
    return {};
  }

  auto& fieldDescr = levelData.fieldDescr;
  ifstream >> fieldDescr.rows >> fieldDescr.cols;
  fieldDescr.data.resize(fieldDescr.rows);
  for (auto &row : fieldDescr.data) {
    row.resize(fieldDescr.cols);
    for (auto &elem : row) {
      ifstream >> elem;

      if ((RoboCommonDefines::SMALL_OBSTACLE_MARKER == elem) ||
          (RoboCommonDefines::BIG_OBSTACLE_MARKER == elem)) {
        ++fieldDescr.obstacleTilesCount;
      } else {
        ++fieldDescr.emptyTilesCount;
      }
    }
  }

  ifstream >> fieldDescr.tileWidth >> fieldDescr.tileHeight;

  auto& robotState = levelData.robotState;
  ifstream >> robotState.fieldPos.row >> robotState.fieldPos.col;
  int32_t robotDirInt {};
  ifstream >> robotDirInt;
  robotState.dir = [robotDirInt](){
    if (robotDirInt >= (getEnumValue(Direction::UP)) &&
        (robotDirInt <= getEnumValue(Direction::LEFT))) {
      return toEnum<Direction>(robotDirInt);
    }
    return Direction::UP;
  }();

  return levelData;
}

std::vector<FieldPos> LevelFileLoader::readMinerLongestSolution(
    const std::string &projectInstallPrefix, int32_t levelId) {
  std::string filePath;
  if (ErrorCode::SUCCESS != readLevelFolder(projectInstallPrefix, levelId,
          filePath)) {
    LOGERR("Error, readLevelFolder() failed");
    return {};
  }

  filePath.append("/").append(MINER_LONGEST_SOLUTION_FILE_NAME);
  std::ifstream ifstream(filePath);
  if (!ifstream) {
    LOGERR("Error, opening file: [%s]. Reason: %s", filePath.c_str(),
        strError().c_str());
    return {};
  }

  size_t tilesCount = 0;
  ifstream >> tilesCount;
  std::vector<FieldPos> sequence(tilesCount);
  for (auto &tile : sequence) {
    ifstream >> tile.row >> tile.col;
  }

  return sequence;
}

ErrorCode LevelFileLoader::readLevelFolder(
    const std::string &projectInstallPrefix, int32_t levelId,
    std::string &outFolderPath) {
  outFolderPath = projectInstallPrefix;
  outFolderPath.append("/").append(ResourceFileHeader::getResourcesFolderName()).append(
      "/levels/level_").append(std::to_string(levelId));

  if (!FileSystemUtils::isDirectoryPresent(outFolderPath)) {
    LOGERR("Error, directory: not present: [%s]", outFolderPath.c_str());
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}
