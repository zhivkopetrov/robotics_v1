#ifndef URSCRIPT_COMMON_URSCRIPTPARSER_H_
#define URSCRIPT_COMMON_URSCRIPTPARSER_H_

//System headers
#include <cstdint>
#include <vector>
#include <string>

//Other libraries headers
#include "utils/ErrorCode.h"

//Own components headers

//Forward declarations

class UrScriptParser {
public:
  UrScriptParser() = delete;

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

#endif /* URSCRIPT_COMMON_URSCRIPTPARSER_H_ */
