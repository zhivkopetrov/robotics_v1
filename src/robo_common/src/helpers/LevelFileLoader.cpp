//Corresponding header
#include "robo_common/helpers/LevelFileLoader.h"

//System headers
#include <cerrno>
#include <cstring>
#include <fstream>

//Other libraries headers
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/file_system/FileSystemUtils.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto FIELD_MAP_FILE_NAME = "field_map.txt";
constexpr auto MINER_LONGEST_SOLUTION_FILE_NAME = "solution.txt";
}

FieldDescription LevelFileLoader::readFieldDescription(
    const std::string &projectInstallPrefix, int32_t levelId) {
  FieldDescription fieldDescr;

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
        strerror(errno));
    return {};
  }

  ifstream >> fieldDescr.rows >> fieldDescr.cols
           >> fieldDescr.tileWidth >> fieldDescr.tileHeight;
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

  return fieldDescr;
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
        strerror(errno));
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
