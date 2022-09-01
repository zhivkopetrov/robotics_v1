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
