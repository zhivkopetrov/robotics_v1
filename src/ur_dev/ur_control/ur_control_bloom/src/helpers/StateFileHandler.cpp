//Corresponding header
#include "ur_control_bloom/helpers/StateFileHandler.h"

//System headers

//Other libraries headers

//Own components headers

ErrorCode StateFileHandler::init(std::string_view filePath) {
  _filePath = filePath;
  if (ErrorCode::SUCCESS != IniFileUtils::parseFile(filePath, _data)) {
    LOGERR("Error in IniFileUtils::parseFile() for [%s]", _filePath.c_str());
    return ErrorCode::FAILURE;
  }

  LOG("\n=================================================================\n"
      "Loading system state:\n");
  IniFileUtils::print(_data);
  LOG("=================================================================\n");
  return ErrorCode::SUCCESS;
}

ErrorCode StateFileHandler::updateEntry(
  const std::string& sectionName, const std::string& entryName,
  const std::string& entryValue) {
  auto sectionIt = _data.find(sectionName);
  if (sectionIt == _data.end()) {
    LOGERR("Section name [%s] not found in file data for [%s]", 
           sectionName.c_str(), _filePath.c_str());
    return ErrorCode::FAILURE;
  }

  IniFileSection& section = sectionIt->second;
  auto entryIt = section.find(entryName);
  if (entryIt == section.end()) {
    LOGERR("Entry name [%s] not found in file data for [%s]", entryName.c_str(),
           _filePath.c_str());
    return ErrorCode::FAILURE;
  }
  entryIt->second = entryValue;

  if (ErrorCode::SUCCESS != IniFileUtils::serializeFile(_filePath, _data)) {
    LOGERR("IniFileUtils::serializeFile() failed for [%s]", _filePath.c_str());
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}
