#ifndef UR_CONTROL_BLOOM_STATEFILEHANDLER_H_
#define UR_CONTROL_BLOOM_STATEFILEHANDLER_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers

//Own components headers
#include "utils/file/IniFileUtils.h"
#include "utils/log/Log.h"

//Forward declarations

inline constexpr auto BOOL_TRUE_VALUE_STR = "True";
inline constexpr auto BOOL_FALSE_VALUE_STR = "False";

class StateFileHandler {
public:
  ErrorCode init(std::string_view filePath);

  //NOTE: if successfuly, the whole file data is flushed to the file system
  ErrorCode updateEntry(
    const std::string& sectionName, const std::string& entryName,
    const std::string& entryValue);

  ErrorCode getEntry(
    const std::string& sectionName, const std::string& entryName, 
    std::string& outValue) const;

private:
  IniFileData _data;
  std::string _filePath;
};

#endif /* UR_CONTROL_BLOOM_STATEFILEHANDLER_H_ */
