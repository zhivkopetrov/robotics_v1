#ifndef ROBO_COMMON_LEVELFILELOADER_H_
#define ROBO_COMMON_LEVELFILELOADER_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"

//Forward declarations

class LevelFileLoader {
public:
  LevelFileLoader() = delete;

  static FieldDescription readFieldDescription(
      const std::string &projectInstallPrefix, int32_t levelId);

  static std::vector<FieldPos> readMinerLongestSolution(
      const std::string &projectInstallPrefix, int32_t levelId);

private:
  static ErrorCode readLevelFolder(const std::string &projectInstallPrefix,
                                   int32_t levelId, std::string &outFolderPath);
};

#endif /* ROBO_COMMON_LEVELFILELOADER_H_ */
