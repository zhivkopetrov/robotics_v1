#ifndef URSCRIPT_COMMOM_URSCRIPTMESSAGEHELPERS_H_
#define URSCRIPT_COMMOM_URSCRIPTMESSAGEHELPERS_H_

//System headers
#include <cstdint>
#include <string>

//Other libraries headers
#include "urscript_interfaces/srv/ur_script.hpp"

//Own components headers
#include "urscript_common/defines/UrScriptDefines.h"

//Forward declarations

/* validates if the payload messages contains a required urscript method,
 * which ends with method 'end'
 * */
bool validateUrscriptServiceRequest(
    const std::shared_ptr<urscript_interfaces::srv::UrScript::Request> &request,
    std::string &outErrorCode, size_t &outEndFindIdx);

std::string extractScriptName(const UrScriptPayload& data);

#endif /* URSCRIPT_COMMOM_URSCRIPTMESSAGEHELPERS_H_ */
