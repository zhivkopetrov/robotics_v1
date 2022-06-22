#ifndef ROBO_COLLECTOR_CONTROLLER_USERAUTHENTICATEHELPER_H_
#define ROBO_COLLECTOR_CONTROLLER_USERAUTHENTICATEHELPER_H_

//System headers
#include <cstdint>
#include <functional>

//Other libraries headers
#include "manager_utils/time/TimerClient.h"
#include "utils/ErrorCode.h"

//Own components headers
#include "robo_collector_controller/helpers/config/UserAuthenticateHelperConfig.h"

//Forward declarations

using InitiateUserAuthenticateCb = std::function<void(const UserData&)>;

class UserAuthenticateHelper final : public TimerClient {
public:
  ErrorCode init(const UserAuthenticateHelperConfig& cfg,
                 const InitiateUserAuthenticateCb& initiateUserAuthenticateCb);

private:
  void onTimeout(const int32_t timerId) override;

  UserData _userData;
  int32_t _timerId { };
  InitiateUserAuthenticateCb _initiateUserAuthenticateCb;
};

#endif /* ROBO_COLLECTOR_CONTROLLER_USERAUTHENTICATEHELPER_H_ */
