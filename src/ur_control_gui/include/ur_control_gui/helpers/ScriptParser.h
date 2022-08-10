#ifndef UR_CONTROL_GUI_SCRIPTPARSER_H_
#define UR_CONTROL_GUI_SCRIPTPARSER_H_

//System headers
#include <cstdint>
#include <vector>
#include <string>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

class ScriptParser {
public:
  ScriptParser() = delete;

  static ErrorCode parseScripts(const std::string &folderLocation,
                                std::vector<std::string> &outScripts);

private:
  static ErrorCode getDirectoryFiles(const std::string &folderLocation,
                                     std::vector<std::string> &outFiles);

  static ErrorCode parseFiles(const std::vector<std::string> &files,
                              std::vector<std::string> &outScripts);
  static ErrorCode parseSingleFile(const std::string &file,
                                   std::string &outScript);
};

#endif /* UR_CONTROL_GUI_SCRIPTPARSER_H_ */
