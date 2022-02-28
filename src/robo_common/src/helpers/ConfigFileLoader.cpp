//Corresponding header
#include "robo_common/helpers/ConfigFileLoader.h"

//C system headers

//C++ system headers
#include <cerrno>
#include <cstring>
#include <fstream>

//Other libraries headers
#include "resource_utils/common/ResourceFileHeader.h"
#include "utils/file_system/FileSystemUtils.h"
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

namespace {
constexpr auto FIELD_MAP_FILE_NAME = "field_map.txt";
constexpr auto MINER_LONGEST_SOLUTION_FILE_NAME = "solution.txt";
}

FieldData ConfigFileLoader::readFieldData(
    const std::string &projectInstallPrefix, int32_t levelId) {
  std::string filePath;
  if (SUCCESS != readLevelFolder(projectInstallPrefix, levelId, filePath)) {
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

  size_t rows = 0;
  size_t cols = 0;
  ifstream >> rows >> cols;
  FieldData data(rows, std::vector<char>(cols));
  for (auto &row : data) {
    for (auto &elem : row) {
      ifstream >> elem;
    }
  }
  return data;
}

std::vector<FieldPos> ConfigFileLoader::readMinerLongestSolution(
    const std::string &projectInstallPrefix, int32_t levelId) {
  std::string filePath;
  if (SUCCESS != readLevelFolder(projectInstallPrefix, levelId, filePath)) {
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

int32_t ConfigFileLoader::readLevelFolder(
    const std::string &projectInstallPrefix, int32_t levelId,
    std::string &outFolderPath) {
  outFolderPath = projectInstallPrefix;
  outFolderPath.append("/").append(ResourceFileHeader::getResourcesFolderName()).append(
      "/levels/level_").append(std::to_string(levelId));

  if (!FileSystemUtils::isDirectoryPresent(outFolderPath)) {
    LOGERR("Error, directory: not present: [%s]", outFolderPath.c_str());
    return FAILURE;
  }

  return SUCCESS;
}
