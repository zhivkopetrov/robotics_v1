//Corresponding header
#include "urscript_bridge/external_api/UrBridgeExternalInterface.h"

//System headers
#include <sstream>
#include <string>
#include <thread>

//Other libraries headers
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "urscript_common/message_helpers/UrScriptMessageHelpers.h"
#include "urscript_common/defines/UrScriptTopics.h"
#include "utils/Log.h"

//Own components headers
#include "urscript_bridge/defines/RobotDefines.h"
#include "urscript_bridge/utils/Tf2Utils.h"

namespace {
using namespace std::literals;

constexpr auto NODE_NAME = "urscript_bridge";

class AtomicBoolAutoLiveScope {
public:
  AtomicBoolAutoLiveScope(std::atomic<bool>& obj) : mObj(obj) {
    mObj = true;
  }
  ~AtomicBoolAutoLiveScope() noexcept {
    mObj = false;
  }

private:
  std::atomic<bool>& mObj;
};

template<typename T>
void waitForPublishers(const T& subscription) {
  while (0 == subscription->get_publisher_count()) {
    LOG("Topic [%s] publishers not available. Waiting 1s ...",
        subscription->get_topic_name());
    std::this_thread::sleep_for(1s);
  }
}

}

UrBridgeExternalInterface::UrBridgeExternalInterface()
    : rclcpp::Node(NODE_NAME), mTcpClient(*this) {

}

ErrorCode UrBridgeExternalInterface::init(
    const UrBridgeExternalInterfaceConfig &cfg) {
  initTogglePinMessagesPayload(cfg.urScriptServiceReadyPin);
  mVerboseLogging = cfg.verboseLogging;

  if (ErrorCode::SUCCESS != mTcpClient.init(cfg.robotIp,
          cfg.robotInterfacePort)) {
    LOGERR("Error, mTcpClient.init() failed");
    return ErrorCode::FAILURE;
  }

  if (ErrorCode::SUCCESS != initCommunication()) {
    LOGERR("Error, initCommunication() failed");
    return ErrorCode::FAILURE;
  }

  mTcpClient.start();

  return ErrorCode::SUCCESS;
}

void UrBridgeExternalInterface::initTogglePinMessagesPayload(uint32_t pin) {
  mUrScriptServiceReadyPin = pin;

  std::ostringstream ss;
  ss << std::setprecision(3);
  ss << "def prepare():\n" << "\tposition_deviation_warning(True, 0.6)\n"
     << "\tset_standard_digital_out(" << mUrScriptServiceReadyPin << ", "
     << "True" << ")\n" << "end\n";
  mTogglePinMsgPayload = ss.str();

  mUntogglePinMsgPayload = "\tset_standard_digital_out(";
  mUntogglePinMsgPayload.append(std::to_string(mUrScriptServiceReadyPin)).
      append(", False)\n");
}

ErrorCode UrBridgeExternalInterface::initCommunication() {
  constexpr auto queueSize = 10;
  const rclcpp::QoS qos(queueSize);

  rclcpp::SubscriptionOptions subsriptionOptions;
  subsriptionOptions.callback_group = mCallbackGroup;

  mIoStatesSubscribtion = create_subscription<IOStates>(
      UR_DRIVER_IO_STATES_TOPIC_NAME, qos,
      std::bind(&UrBridgeExternalInterface::handleIOState, this,
          std::placeholders::_1), subsriptionOptions);

  mUrScriptSubscribtion = create_subscription<String>(URSCRIPT_TOPIC, qos,
      std::bind(&UrBridgeExternalInterface::handleUrScript, this,
          std::placeholders::_1), subsriptionOptions);

  mUrScriptService = create_service<UrScriptSrv>(URSCRIPT_SERVICE,
      std::bind(&UrBridgeExternalInterface::handleUrScriptService, this,
          std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, mCallbackGroup);

  mUrScriptServicePreempt = create_service<Trigger>(URSCRIPT_SERVICE_PREEMPT,
      std::bind(&UrBridgeExternalInterface::handleUrScriptServicePreempt, this,
          std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, mCallbackGroup);

  mGetEefAngleAxisService = create_service<GetEefAngleAxis>(
      GET_EEF_ANGLE_AXIS_SERVICE,
      std::bind(&UrBridgeExternalInterface::handleGetEefAngleAxisService, this,
          std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, mCallbackGroup);

  mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  mTfListener = std::make_unique<tf2_ros::TransformListener>(*mTfBuffer);

  waitForPublishers(mIoStatesSubscribtion);

  return ErrorCode::SUCCESS;
}

void UrBridgeExternalInterface::handleIOState(
    const IOStates::SharedPtr ioStates) {
  std::lock_guard<Mutex> lock(mIoMutex);
  mLatestIoStates = *ioStates;
}

void UrBridgeExternalInterface::handleUrScript(
    const String::SharedPtr urScript) {

  if (mVerboseLogging) {
    const std::string scriptName = extractScriptName(urScript->data);
    LOG_T("Received UrScript Topic: [%s]. Sending to robot\n", 
        scriptName.c_str());
  }
  mTcpClient.send(urScript->data);
}

void UrBridgeExternalInterface::handleUrScriptService(
    const std::shared_ptr<UrScriptSrv::Request> request,
    std::shared_ptr<UrScriptSrv::Response> response) {
  response->success = true;
  AtomicBoolAutoLiveScope urscriptServiceCall(mActiveUrscriptServiceCall);
  [[maybe_unused]]std::string scriptName;
  if (mVerboseLogging) {
    scriptName = extractScriptName(request->data);
    LOG_T("Received UrScript Service: [%s]", scriptName.c_str());
  }

  size_t endDelimiterFindIdx { };
  const bool success = validateUrscriptServiceRequest(request,
      response->error_reason, endDelimiterFindIdx);
  if (!success) {
    response->success = false;
    LOGERR("%s", response->error_reason.c_str());
    return;
  }

  if (mVerboseLogging) {
    LOG_T("UrScript Service: [%s] - expecting command pin(TOGGLED) from robot", 
          scriptName.c_str());
  }

  mTcpClient.send(mTogglePinMsgPayload);
  bool waitAborted = waitForPinState(PinState::TOGGLED);
  if (waitAborted) {
    if (mVerboseLogging) {
      LOG_T("UrScript Service: [%s] - aborted externally", scriptName.c_str());
    }
    return;
  }
  if (mVerboseLogging) {
    LOG_T("UrScript Service: [%s] - command pin(TOGGLED) received from robot\n",
          scriptName.c_str());
  }

  std::string data = request->data;
  data.insert(endDelimiterFindIdx, mUntogglePinMsgPayload);

  if (mVerboseLogging) {
    LOG_T("UrScript Service: [%s] - expecting command pin(UNTOGGLED) "
          "from robot",scriptName.c_str());
  }
  mTcpClient.send(data);
  waitAborted = waitForPinState(PinState::UNTOGGLED);
  if (waitAborted) {
    if (mVerboseLogging) {
      LOG_T("UrScript Service: [%s] - aborted externally", scriptName.c_str());
    }
    return;
  }
  if (mVerboseLogging) {
    LOG_T("UrScript Service: [%s] - command pin(UNTOGGLED) received "
          "from robot\n", scriptName.c_str());
  }
}

void UrBridgeExternalInterface::handleUrScriptServicePreempt(
  [[maybe_unused]]const std::shared_ptr<Trigger::Request> request,
  std::shared_ptr<Trigger::Response> response) {
  response->success = true;
  if (!mActiveUrscriptServiceCall) {
    return;
  }
    
  AtomicBoolAutoLiveScope preemptRequest(mActiveUrscriptServicePreemptRequest);

  //wait for service to be aborted
  while (true) {
    std::this_thread::sleep_for(10ms);
    if (!mActiveUrscriptServiceCall) {
      break;
    }
  }
}

void UrBridgeExternalInterface::handleGetEefAngleAxisService(
    [[maybe_unused]]const std::shared_ptr<GetEefAngleAxis::Request> request,
    std::shared_ptr<GetEefAngleAxis::Response> response) {
  std::lock_guard<Mutex> lock(mTfMutex);
  try {
    const geometry_msgs::msg::TransformStamped trasnformStamped =
        mTfBuffer->lookupTransform(ur_links::BASE_NAME, ur_links::TOOL0_NAME,
            tf2::TimePointZero);
    response->success = true;
    getAngleAxisRepresentation(trasnformStamped.transform.rotation,
        response->angle_axis);
  } catch (const tf2::TransformException &ex) {
    response->success = false;
    response->error_reason = "Could not transform ";
    response->error_reason.append(ur_links::BASE_NAME).append(" to ").append(
        ur_links::TOOL0_NAME).append(", because: ").append(ex.what());
    LOGERR("%s", response->error_reason.c_str());
  }
}

bool UrBridgeExternalInterface::waitForPinState(PinState state) {
  const bool waitCondidition = PinState::TOGGLED == state ? true : false;
  bool waitAborted = false; 
  while (true) {
    if (mActiveUrscriptServicePreemptRequest) {
      waitAborted = true;
      return waitAborted;
    }

    {
      std::lock_guard<Mutex> lock(mIoMutex);
      if (waitCondidition == 
          mLatestIoStates.digital_out_states[mUrScriptServiceReadyPin].state) {
        break;
      }
    }

    std::this_thread::sleep_for(10ms);
  }

  return waitAborted;
}

