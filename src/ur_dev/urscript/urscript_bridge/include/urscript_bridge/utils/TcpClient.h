#ifndef URSCRIPT_BRIDGE_TCPCLIENT_H
#define URSCRIPT_BRIDGE_TCPCLIENT_H

//System headers
#include <vector>
#include <string>
#include <queue>

//Other libraries headers
#include <rclcpp/logger.hpp>
#include <std_msgs/msg/string.hpp>

#include <boost/thread.hpp>
#include <boost/asio.hpp>

//Own components headers
#include "utils/class/NonCopyable.h"
#include "utils/class/NonMoveable.h"

//Forward declarations
namespace rclcpp {
class Node;
} //namespace rclcpp

class TcpClient: public NonCopyable, public NonMoveable {
public:
  TcpClient(rclcpp::Node &node);
  ~TcpClient() noexcept;

  void start(const std::string &address, uint16_t port);
  void stop();

  void send(const std::string &data);

private:
  void run();
  void doConnect();
  void doClose();
  void doWrite(const std::string &data);
  void connectHandler(
      const boost::shared_ptr<boost::asio::ip::tcp::socket> &socket,
      const boost::system::error_code &error);
  void connectTimeoutHandler(const boost::system::error_code &error);
  void writeHandler(const boost::system::error_code &error,
                    size_t bytesWritten);

  enum class State {
    Disconnected, Connecting, Connected
  };

  rclcpp::Node &mNode;
  rclcpp::Logger mLogger;
  std::atomic<State> mState = State::Disconnected;
  uint16_t mPort = 0;
  boost::atomic_bool mRunning = false;
  std::string mAddress;

  boost::mutex mMutex;
  boost::thread mThread;
  boost::asio::io_service mIOService;
  boost::shared_ptr<boost::asio::ip::tcp::socket> mSocket;
  boost::asio::io_service::work mWork;
  boost::asio::deadline_timer mConnectTimer;

  std::queue<std::string> mQueue;
};

#endif /* URSCRIPT_BRIDGE_TCPCLIENT_H */
