//Corresponding header
#include "urscript_bridge/utils/TcpClient.h"

//System headers

//Other libraries headers
#include <rclcpp/node.hpp>

//Own components headers

using namespace boost::asio::ip;

static void invokeCallback() {
}

TcpClient::TcpClient(rclcpp::Node &node)
    : mNode(node), mLogger(node.get_logger()),
      mWork(mIOService),
      mConnectTimer(mIOService) {
}

TcpClient::~TcpClient() noexcept {
  stop();
}

ErrorCode TcpClient::init(const std::string &address, uint16_t port) {
  mPort = port;
  mAddress = address;

  return ErrorCode::SUCCESS;
}

void TcpClient::start() {
  if (mRunning) {
    return;
  }

  mState = TcpClient::State::Disconnected;
  mRunning = true;
  mThread = boost::thread(&TcpClient::run, this);
}

void TcpClient::stop() {
  mRunning = false;
  mIOService.post(invokeCallback);

  if (mThread.joinable()) {
    mThread.join();
  }
}

void TcpClient::send(const std::string &data) {
  if (!mRunning || mState != TcpClient::State::Connected) {
    return;
  }

  {
    boost::lock_guard<boost::mutex> lock(mMutex);
    mQueue.push(data);
  }

  mIOService.post(invokeCallback);
}

void TcpClient::run() {
  while (mRunning) {
    doConnect();
    try {
      mIOService.run_for(std::chrono::milliseconds(50));
      mIOService.reset();
    } catch (const boost::exception&) {
    }

    boost::lock_guard<boost::mutex> lock(mMutex);
    if (mQueue.size() == 1) {
      doWrite(mQueue.front());
    }
  }

  doClose();
}

void TcpClient::doConnect() {
  if (mState != TcpClient::State::Disconnected) {
    return;
  }

  RCLCPP_INFO_STREAM_THROTTLE(mLogger, *mNode.get_clock(), 1000,
                              "Connecting on IP: [" << mAddress <<
                              "], port: [" << mPort << "] ...");

  mState = TcpClient::State::Connecting;
  mSocket.reset(new tcp::socket(mIOService));
  mSocket->async_connect(
      tcp::endpoint(boost::asio::ip::address::from_string(mAddress), mPort),
      boost::bind(&TcpClient::connectHandler, this, mSocket,
          boost::asio::placeholders::error));

  mConnectTimer.expires_from_now(boost::posix_time::seconds(10));
  mConnectTimer.async_wait(
      boost::bind(&TcpClient::connectTimeoutHandler, this,
          boost::asio::placeholders::error));
}

void TcpClient::doClose() {
  if (mState == TcpClient::State::Disconnected) {
    return;
  }

  RCLCPP_INFO_THROTTLE(mLogger, *mNode.get_clock(), 1000, "Closing socket...");

  mConnectTimer.expires_at(boost::posix_time::pos_infin);
  try {
    mSocket->shutdown(boost::asio::socket_base::shutdown_both);
  } catch (const std::exception&) {
  }
  mSocket->close();
  mSocket.reset();
  mState = TcpClient::State::Disconnected;

  mQueue = std::queue<std::string>{};
}

void TcpClient::doWrite(const std::string &data) {
  boost::asio::async_write(*mSocket,
      boost::asio::buffer(data.data(), data.size()),
      boost::asio::transfer_all(),
      boost::bind(&TcpClient::writeHandler, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}

void TcpClient::connectHandler(
    [[maybe_unused]]const boost::shared_ptr<tcp::socket> &socket,
    const boost::system::error_code &error) {
  if (error) {
    RCLCPP_INFO_THROTTLE(mLogger, *mNode.get_clock(), 1000, "Connect error!");

    doClose();
  } else {
    mConnectTimer.expires_at(boost::posix_time::pos_infin);
    mState = TcpClient::State::Connected;
    RCLCPP_INFO(mLogger, "Connected...");
  }
}

void TcpClient::connectTimeoutHandler(const boost::system::error_code &error) {
  if (error && error != boost::asio::error::operation_aborted) {
    RCLCPP_INFO(mLogger, "Connected timeout...");
    doClose();
  }
}

void TcpClient::writeHandler(const boost::system::error_code &error,
                             [[maybe_unused]]size_t bytesWritten) {
  if (error) {
    RCLCPP_INFO(mLogger, "Write error!");
    doClose();
  }

  boost::lock_guard<boost::mutex> lock(mMutex);
  mQueue.pop();
  if (!mQueue.empty()) {
    doWrite(mQueue.front());
  }
}

