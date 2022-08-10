//Corresponding header
#include "ur_control_gui/helpers/ScriptParser.h"

//System headers
#include <fstream>
#include <cstring>
#include <cerrno>

//Other libraries headers
#include "utils/file_system/FileSystemUtils.h"

//Own components headers
#include "utils/Log.h"

ErrorCode ScriptParser::parseScripts(const std::string &folderLocation,
                                     std::vector<std::string> &outScripts) {
  std::vector<std::string> files;
  ErrorCode err = ScriptParser::getDirectoryFiles(folderLocation, files);
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, ScriptParser::getDirectoryFiles() failed for directory [%s]",
        folderLocation.c_str());
    return ErrorCode::FAILURE;
  }

  err = parseFiles(files, outScripts);
  if (ErrorCode::SUCCESS != err) {
    LOGERR("Error, ScriptParser::parseFiles() failed for directory [%s]",
        folderLocation.c_str());
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode ScriptParser::getDirectoryFiles(const std::string &folderLocation,
                                          std::vector<std::string> &outFiles) {
  if (!FileSystemUtils::isDirectoryPresent(folderLocation)) {
    LOGERR("Error, script folder [%s] is missing", folderLocation.c_str());
    return ErrorCode::FAILURE;
  }

  const std::vector<std::string> blackListFolders { };
  const ErrorCode err = FileSystemUtils::getAllFilesInDirectoryRecursively(
      folderLocation, blackListFolders, outFiles);
  if (ErrorCode::SUCCESS != err) {
    LOGERR(
        "Error, getAllFilesInDirectoryRecursively() failed for directory [%s]",
        folderLocation.c_str());
    return ErrorCode::FAILURE;
  }

  return ErrorCode::SUCCESS;
}

ErrorCode ScriptParser::parseFiles(const std::vector<std::string> &files,
                                   std::vector<std::string> &outScripts) {
  const size_t size = files.size();
  outScripts.resize(size);

  for (size_t i = 0; i < size; ++i) {
    const ErrorCode err = parseSingleFile(files[i], outScripts[i]);
    if (ErrorCode::SUCCESS != err) {
      LOGERR("Error, parseSingleFile() failed for file [%s]", files[i].c_str());
      return ErrorCode::FAILURE;
    }
  }

  return ErrorCode::SUCCESS;
}

ErrorCode ScriptParser::parseSingleFile(const std::string &file,
                                        std::string &outScript) {
  std::ifstream ifstream(file, std::ios::in);

  if (!ifstream) {
    LOGERR("Error, opening file [%s]. Reason: [%s]", file.c_str(),
        strerror(errno));
    return ErrorCode::FAILURE;
  }

  std::string line;
  while (true) {
    getline(ifstream, line);
    if (line.empty()) {
      break;
    }

    outScript.append(line).append("\n");
    line.clear();
  }

  return ErrorCode::SUCCESS;
}
