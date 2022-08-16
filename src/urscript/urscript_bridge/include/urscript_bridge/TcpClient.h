#ifndef URSCRIPT_INTERFACE_TCPCLIENT_H
#define URSCRIPT_INTERFACE_TCPCLIENT_H

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <boost/thread.hpp>
#include <boost/asio.hpp>

class TcpClient {
public:
  enum class State {
    Disconnected, Connecting, Connected
  };

  TcpClient(rclcpp::Node &node);

  // Disable copy and move
  TcpClient(const TcpClient &other) = delete;
  TcpClient(TcpClient &&other) noexcept = delete;

  ~TcpClient() noexcept = default;

  // Disable copy and move
  TcpClient& operator=(const TcpClient &other) = delete;
  TcpClient& operator=(TcpClient &&other) noexcept = delete;

  void start(const std::string &address, uint16_t port);
  void stop();

  void send(const std::string &data);

private:
  void run();
  void doConnect();
  void doClose();
  void doWrite(const std::string &data);
  void connectHandler(
      const boost::shared_ptr<boost::asio::ip::tcp::socket>& /*socket*/,
      const boost::system::error_code &error);
  void connectTimeoutHandler(const boost::system::error_code &error);
  void writeHandler(const boost::system::error_code &error,
                    std::size_t /*bytesWritten*/);

  rclcpp::Node &mNode;
  rclcpp::Logger mLogger;
  std::atomic<TcpClient::State> mState;
  uint16_t mPort;
  std::string mAddress;
  boost::atomic_bool mRunning;
  boost::mutex mMutex;
  boost::thread mThread;
  boost::asio::io_service mIOService;
  boost::shared_ptr<boost::asio::ip::tcp::socket> mSocket;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> mWorkGuard;
  boost::asio::deadline_timer mConnectTimer;
  std::vector<std::string> mQueue;
  std::vector<std::string> mInternalQueue;
};

#endif /* URSCRIPT_INTERFACE_TCPCLIENT_H */
