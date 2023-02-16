#ifndef UR_CONTROL_BLOOM_STATEFILEHANDLER_H_
#define UR_CONTROL_BLOOM_STATEFILEHANDLER_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers

//Own components headers
#include "utils/file/IniFileUtils.h"
#include "utils/Log.h"

//Forward declarations

class StateFileHandler {
public:
  ErrorCode init(std::string_view filePath);

  //NOTE: if successfuly, the whole file data is flushed to the file system
  ErrorCode updateEntry(
    const std::string& sectionName, const std::string& entryName,
    const std::string& entryValue);

private:
  IniFileData _data;
  std::string _filePath;
};

#endif /* UR_CONTROL_BLOOM_STATEFILEHANDLER_H_ */
