//Corresponding header
#include "robo_collector_controller/helpers/UserAuthenticateHelper.h"

//System headers

//Other libraries headers
#include "utils/Log.h"

//Own components headers

ErrorCode UserAuthenticateHelper::init(
    const UserAuthenticateHelperConfig &cfg,
    const InitiateUserAuthenticateCb &initiateUserAuthenticateCb) {
  if (nullptr == initiateUserAuthenticateCb) {
    LOGERR("Error, nullptr provided for InitiateUserAuthenticateCb");
    return ErrorCode::FAILURE;
  }
  _initiateUserAuthenticateCb = initiateUserAuthenticateCb;
  _userData = cfg.userData;
  _timerId = cfg.timerId;

  //during initialization the ROS2 executor still has not been started
  //initiate the user authentication with some delay
  //after the application main loop has begun
  constexpr int64_t intervalMs = 1000;
  startTimer(intervalMs, _timerId, TimerType::ONESHOT);

  return ErrorCode::SUCCESS;
}

void UserAuthenticateHelper::onTimeout(const int32_t timerId) {
  if (timerId == _timerId) {
    _initiateUserAuthenticateCb(_userData);
  } else {
    LOGERR("Error, received unsupported timerId: %d", timerId);
  }
}
