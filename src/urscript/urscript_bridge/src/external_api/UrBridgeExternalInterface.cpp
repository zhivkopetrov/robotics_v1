//Corresponding header
#include "urscript_bridge/external_api/UrBridgeExternalInterface.h"

//System headers
#include <sstream>
#include <string>

//Other libraries headers

//Own components headers

namespace {
constexpr auto NODE_NAME = "urscript_bridge";
}

UrBridgeExternalInterface::UrBridgeExternalInterface(
    const rclcpp::NodeOptions &options)
    : rclcpp::Node(NODE_NAME, options), mTcpClient(*this) {
  std::string scriptTopic;
  std::string scriptService;
  std::string robotIp;
  uint16_t robotPort;

  get_parameter("script_topic", scriptTopic);
  get_parameter("script_service", scriptService);
  get_parameter("robot_ip", robotIp);
  get_parameter("robot_port", robotPort);
  get_parameter("robot_pin", mRobotPin);

  const rclcpp::CallbackGroup::SharedPtr callbackGroup = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions subsriptionOptions;
  subsriptionOptions.callback_group = callbackGroup;

  mIoStatesSubscribtion = create_subscription<IOStates>(
      "io_and_status_controller/io_states", 10,
      std::bind(&UrBridgeExternalInterface::handleIOState, this,
          std::placeholders::_1), subsriptionOptions);

  mUrScriptSubscribtion = create_subscription<String>(scriptTopic, 10,
      std::bind(&UrBridgeExternalInterface::handleUrScript, this,
          std::placeholders::_1), subsriptionOptions);

  mUrScriptService = create_service<UrScriptSrv>(scriptService,
      std::bind(&UrBridgeExternalInterface::handleUrScriptService, this,
          std::placeholders::_1, std::placeholders::_2), { }, callbackGroup);

  mTcpClient.start(robotIp, robotPort);
}

void UrBridgeExternalInterface::handleIOState(
    const IOStates::SharedPtr ioStates) {
  std::lock_guard<Mutex> lock(mMutex);
  mlatestIoStates = *ioStates;
}

void UrBridgeExternalInterface::handleUrScript(
    const String::SharedPtr urScript) {
  std::cout << "Sending data to robot:\n" << urScript->data << std::endl;
  mTcpClient.send(urScript->data);
}

void UrBridgeExternalInterface::handleUrScriptService(
    const std::shared_ptr<UrScriptSrv::Request> request,
    std::shared_ptr<UrScriptSrv::Response> response) {
  std::cout << "Received data:\n" << request->data << std::endl;
  constexpr const char *endDelimiter = "end";
  const size_t endDelimiterStartIdx = request->data.rfind(endDelimiter);
  if (std::string::npos == endDelimiterStartIdx) {
    std::cout << "Error, [" << endDelimiter << "] delimiter not found"
              << std::endl;
    response->ok = false;
    return;
  }

  std::ostringstream ss;
  ss << std::setprecision(3);
  ss << "def prepare():\n" << "\tposition_deviation_warning(True, 0.6)\n"
     << "\tset_standard_digital_out(" << static_cast<uint32_t>(mRobotPin)
     << ", " << "True" << ")\n" << "end\n";

  std::cout << "Sending to robot: " << ss.str() << std::endl;

  using namespace std::literals;

  mTcpClient.send(ss.str());

  std::cout << "Waiting for flipped bit: " << std::endl;

  for (;;) {
    {
      std::lock_guard<Mutex> lock(mMutex);
      if (mlatestIoStates.digital_out_states[mRobotPin].state == true) {
        break;
      }
    }

    std::this_thread::sleep_for(10ms);
  }

  constexpr const char *TAB = "  ";

  std::string data = request->data;
  std::string pinPostfix = TAB;
  pinPostfix.append("set_standard_digital_out(").append(
      std::to_string(static_cast<uint32_t>(mRobotPin))).append(", False)\n");

  data.insert(endDelimiterStartIdx, pinPostfix);

  std::cout << "Sending to robot:\n" << data << std::endl;

  mTcpClient.send(data);

  std::cout << "Waiting for UNflipped bit: " << std::endl;

  for (;;) {
    {
      std::lock_guard<Mutex> lock(mMutex);
      if (mlatestIoStates.digital_out_states[mRobotPin].state == false) {
        break;
      }
    }

    std::this_thread::sleep_for(10ms);
  }

  std::cout << "Bit was UNflipped: " << std::endl;

  response->ok = true;
}

