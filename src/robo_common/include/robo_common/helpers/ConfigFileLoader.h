#ifndef ROBO_COMMON_CONFIGFILELOADER_H_
#define ROBO_COMMON_CONFIGFILELOADER_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <string>

//Other libraries headers

//Own components headers
#include "robo_common/defines/RoboCommonDefines.h"

//Forward declarations

class ConfigFileLoader {
public:
  ConfigFileLoader() = delete;

  static FieldData readFieldData(const std::string &projectInstallPrefix,
                                 int32_t levelId);

  static std::vector<FieldPos> readMinerLongestSolution(
      const std::string &projectInstallPrefix, int32_t levelId);

private:
  static int32_t readLevelFolder(const std::string &projectInstallPrefix,
                                 int32_t levelId, std::string &outFolderPath);
};

#endif /* ROBO_COMMON_CONFIGFILELOADER_H_ */
