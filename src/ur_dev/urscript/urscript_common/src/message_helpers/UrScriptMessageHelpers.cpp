//Corresponding header
#include "urscript_common/message_helpers/UrScriptMessageHelpers.h"

//System headers

//Other libraries headers

//Own components headers

bool validateUrscriptServiceRequest(
    const std::shared_ptr<urscript_interfaces::srv::UrScript::Request> &request,
    std::string &outErrorCode, size_t &outEndFindIdx) {
  constexpr const char *endDelimiter = "end";
  outEndFindIdx = request->data.rfind(endDelimiter);
  if (std::string::npos == outEndFindIdx) {
    outErrorCode = "Error, [";
    outErrorCode.append(endDelimiter).append(
        "] delimiter not found in request payload: ").append(request->data);
    return false;
  }

  return true;
}

std::string extractScriptName(const UrScriptPayload& data) {
  constexpr auto methodDefinitionStart = "def ";
  const size_t medtodDefinitionStartPos = data.find(methodDefinitionStart);
  if (std::string::npos == medtodDefinitionStartPos) {
    return "unnamed";
  }

  constexpr auto methodParamDefinitionEnd = "):";
  const size_t methodParamDefinitionEndPos = 
    data.find(methodParamDefinitionEnd);
  if (std::string::npos == methodParamDefinitionEndPos) {
    return "unnamed";
  }

  constexpr auto methodParamDefinitionStart = "(";
  const size_t methodParamDefinitionStartPos = 
    data.find(methodParamDefinitionStart);
  if (std::string::npos == methodParamDefinitionStartPos) {
    return "unnamed";
  }

  constexpr size_t methodDefinitionStartSize = 4;
  return data.substr(
    methodDefinitionStartSize, 
    methodParamDefinitionStartPos - methodDefinitionStartSize);
}
