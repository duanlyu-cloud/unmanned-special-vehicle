#include "ar_hardware_interface/teensy_driver.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <sys/select.h>
#include <thread>

#define FW_VERSION "0.0.1"

namespace ar_hardware_interface {

void TeensyDriver::init(std::string port, int baudrate, int num_joints) {
  // @TODO read version from config
  version_ = FW_VERSION;

  // initialise joint and encoder calibration (before exchange to avoid segfault)
  num_joints_ = num_joints;
  joint_positions_deg_.resize(num_joints_);
  enc_calibrations_.resize(num_joints_);

  // establish connection with teensy board
  boost::system::error_code ec;
  serial_port_.open(port, ec);

  if (ec) {
    RCLCPP_WARN(logger_, "Failed to connect to serial port %s: %s", port.c_str(), ec.message().c_str());
    connected_ = false;
    return;
  } else {
    connected_ = true;
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(
        static_cast<uint32_t>(baudrate)));
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    RCLCPP_INFO(logger_, "Successfully connected to serial port %s",
                port.c_str());
  }

  // flush stale serial data from previous session
  try {
    serial_port_.cancel();
  } catch (...) {}
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  drainBuffer();

  initialised_ = false;
  std::string msg = "STA" + version_ + "\n";

  while (!initialised_) {
    RCLCPP_INFO(logger_, "Waiting for response from Teensy on port %s",
                port.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    exchange(msg);
  }
  RCLCPP_INFO(logger_, "Successfully initialised driver on port %s",
              port.c_str());
}

TeensyDriver::TeensyDriver() : serial_port_(io_service_) {}

void TeensyDriver::setStepperSpeed(std::vector<double>& max_speed,
                                   std::vector<double>& max_accel) {
  std::string outMsg = "SS";
  for (int i = 0, charIdx = 0; i < num_joints_; ++i, charIdx += 2) {
    outMsg += 'A' + charIdx;
    outMsg += std::to_string(max_speed[i]);
    outMsg += 'A' + charIdx + 1;
    outMsg += std::to_string(max_accel[i]);
  }
  outMsg += "\n";
  exchange(outMsg);
}

// Update between hardware interface and hardware driver
void TeensyDriver::update(std::vector<double>& pos_commands,
                          std::vector<double>& joint_positions) {
  // construct update message
  std::string outMsg = "MT";
  for (int i = 0; i < num_joints_; ++i) {
    outMsg += 'A' + i;
    outMsg += std::to_string(pos_commands[i]);
  }
  outMsg += "\n";

  // run the communication with board
  exchange(outMsg);

  joint_positions = joint_positions_deg_;
}

void TeensyDriver::calibrateJoints() {
  // 串行校准：先校准关节 1-3 (A,B,C)，完成后回到 REST，再校准关节 4-6 (D,E,F)
  // 避免所有关节并行运动导致的中间姿态损坏末端结构
  RCLCPP_INFO(logger_, "Calibrating joints 1-3 (A,B,C)...");
  std::string outMsg1 = "JC1\n";
  exchangeWithTimeout(outMsg1, CALIBRATION_TIMEOUT_MS);

  RCLCPP_INFO(logger_, "Calibrating joints 4-6 (D,E,F)...");
  std::string outMsg2 = "JC2\n";
  exchangeWithTimeout(outMsg2, CALIBRATION_TIMEOUT_MS);
}

void TeensyDriver::toggleClosedLoop() {
  if (!connected_) return;
  std::lock_guard<std::mutex> lock(serial_mutex_);
  std::string outMsg = "CL\n";
  std::string err;

  if (!transmit(outMsg, err)) {
    RCLCPP_ERROR(logger_, "toggleClosedLoop: transmit error: %s", err.c_str());
    return;
  }

  std::string inMsg;
  if (!receive(inMsg, DEFAULT_RECEIVE_TIMEOUT_MS)) {
    RCLCPP_ERROR(logger_, "toggleClosedLoop: receive timeout");
    return;
  }

  if (inMsg == "CLA1") {
    closed_loop_enabled_ = true;
    RCLCPP_INFO(logger_, "Closed-loop ENABLED (encoder feedback active)");
  } else if (inMsg == "CLA0") {
    closed_loop_enabled_ = false;
    RCLCPP_INFO(logger_, "Closed-loop DISABLED (open-loop stepper only)");
  } else {
    RCLCPP_WARN(logger_, "toggleClosedLoop: unexpected response '%s'",
                inMsg.c_str());
  }
}

void TeensyDriver::setClosedLoopDesired(bool enable) {
  if (!connected_ || enable == closed_loop_enabled_) return;
  toggleClosedLoop();
}

void TeensyDriver::getJointPositions(std::vector<double>& joint_positions) {
  // get current joint positions
  std::string msg = "JP\n";
  exchange(msg);
  joint_positions = joint_positions_deg_;
}

// Send specific commands
void TeensyDriver::sendCommand(std::string outMsg) { exchange(outMsg); }

// Send msg to board and collect data
void TeensyDriver::exchange(std::string outMsg) {
  exchangeWithTimeout(outMsg, DEFAULT_RECEIVE_TIMEOUT_MS);
}

// Send msg to board and collect data with configurable receive timeout
void TeensyDriver::exchangeWithTimeout(std::string outMsg,
                                       int receive_timeout_ms) {
  std::lock_guard<std::mutex> lock(serial_mutex_);

  // Guard: do not attempt communication on a disconnected port
  if (!connected_) {
    RCLCPP_WARN_THROTTLE(logger_, clock_, 5000,
                         "exchangeWithTimeout: serial port disconnected, "
                         "skipping TX/RX");
    return;
  }

  std::string inMsg;
  std::string errTransmit = "";

  if (!transmit(outMsg, errTransmit)) {
    RCLCPP_ERROR(logger_, "Error in transmit: %s", errTransmit.c_str());
    return;
  }

  bool done = false;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(receive_timeout_ms);

  while (!done) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      RCLCPP_ERROR(logger_,
                   "exchangeWithTimeout: total timeout after %d ms",
                   receive_timeout_ms);
      return;
    }
    int remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                           deadline - now)
                           .count();

    if (!receive(inMsg, remaining_ms)) {
      RCLCPP_ERROR(logger_, "exchangeWithTimeout: receive timed out");
      return;
    }

    // parse msg
    std::string header = inMsg.substr(0, 2);
    if (header == "ST") {
      checkInit(inMsg);
      done = true;
    } else if (header == "JC") {
      updateEncoderCalibrations(inMsg);
      done = true;
    } else if (header == "JP") {
      updateJointPositions(inMsg);
      done = true;
    } else if (header == "DB") {
    } else {
      RCLCPP_WARN(logger_, "Unknown header '%s', continuing to wait",
                  header.c_str());
    }
  }
}

bool TeensyDriver::transmit(std::string msg, std::string& err) {
  boost::system::error_code ec;
  const auto sendBuffer = boost::asio::buffer(msg.c_str(), msg.size());

  boost::asio::write(serial_port_, sendBuffer, ec);

  if (!ec) {
    return true;
  } else {
    err = "Error in transmit";
    // Terminal errors: device disconnected — mark as disconnected to stop
    // further attempts and prevent error spam at 100Hz read() cycle
    if (ec == boost::asio::error::eof ||
        ec == boost::asio::error::bad_descriptor) {
      RCLCPP_ERROR(logger_, "transmit: serial port lost (EOF/bad fd), "
                   "marking disconnected");
      connected_ = false;
    }
    return false;
  }
}

bool TeensyDriver::receive(std::string& inMsg, int timeout_ms) {
  char c;
  std::string msg = "";
  bool eol = false;

  auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (!eol) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return false;
    }
    auto remaining_us = std::chrono::duration_cast<std::chrono::microseconds>(
        deadline - now);

    boost::asio::serial_port::native_handle_type native_fd =
        serial_port_.native_handle();
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(native_fd, &read_fds);

    struct timeval tv;
    tv.tv_sec = remaining_us.count() / 1000000;
    tv.tv_usec = remaining_us.count() % 1000000;

    int ret =
        select(static_cast<int>(native_fd) + 1, &read_fds, nullptr, nullptr,
               &tv);
    if (ret < 0) {
      RCLCPP_ERROR(logger_, "receive: select() error: %s", std::strerror(errno));
      return false;
    }
    if (ret == 0) {
      return false;
    }

    boost::system::error_code ec;
    boost::asio::read(serial_port_, boost::asio::buffer(&c, 1), ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "receive: read error: %s", ec.message().c_str());
      // Terminal errors: device disconnected — mark as disconnected
      if (ec == boost::asio::error::eof ||
          ec == boost::asio::error::bad_descriptor) {
        RCLCPP_ERROR(logger_, "receive: serial port lost (EOF/bad fd), "
                     "marking disconnected");
        connected_ = false;
      }
      return false;
    }

    switch (c) {
      case '\r':
        break;
      case '\n':
        eol = true;
        break;
      default:
        msg += c;
    }
  }
  inMsg = msg;
  return true;
}

void TeensyDriver::drainBuffer() {
  boost::system::error_code ec;
  char dummy;
  int drained = 0;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(200);

  while (true) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;

    boost::asio::serial_port::native_handle_type native_fd =
        serial_port_.native_handle();
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(native_fd, &read_fds);

    struct timeval tv = {0, 10000};
    int ret =
        select(static_cast<int>(native_fd) + 1, &read_fds, nullptr, nullptr,
               &tv);
    if (ret <= 0) break;

    size_t n = serial_port_.read_some(boost::asio::buffer(&dummy, 1), ec);
    if (ec || n == 0) break;
    drained++;
  }
  if (drained > 0) {
    RCLCPP_INFO(logger_, "drainBuffer: flushed %d stale bytes", drained);
  }
}

void TeensyDriver::checkInit(std::string msg) {
  std::size_t ack_idx = msg.find("A", 2) + 1;
  std::size_t version_idx = msg.find("B", 2) + 1;
  int ack = std::stoi(msg.substr(ack_idx, version_idx));
  if (ack) {
    initialised_ = true;
  } else {
    std::string version = msg.substr(version_idx);
    RCLCPP_ERROR(logger_, "Firmware version mismatch %s", version.c_str());
  }
}

void TeensyDriver::updateEncoderCalibrations(std::string msg) {
  size_t idx1 = msg.find("A", 2) + 1;
  size_t idx2 = msg.find("B", 2) + 1;
  size_t idx3 = msg.find("C", 2) + 1;
  size_t idx4 = msg.find("D", 2) + 1;
  size_t idx5 = msg.find("E", 2) + 1;
  size_t idx6 = msg.find("F", 2) + 1;
  enc_calibrations_[0] = std::stoi(msg.substr(idx1, idx2 - idx1));
  enc_calibrations_[1] = std::stoi(msg.substr(idx2, idx3 - idx2));
  enc_calibrations_[2] = std::stoi(msg.substr(idx3, idx4 - idx3));
  enc_calibrations_[3] = std::stoi(msg.substr(idx4, idx5 - idx4));
  enc_calibrations_[4] = std::stoi(msg.substr(idx5, idx6 - idx5));
  enc_calibrations_[5] = std::stoi(msg.substr(idx6));

  // @TODO update config file
  RCLCPP_INFO(logger_, "Successfully updated encoder calibrations");
}

void TeensyDriver::updateJointPositions(std::string msg) {
  size_t idx1 = msg.find("A", 2) + 1;
  size_t idx2 = msg.find("B", 2) + 1;
  size_t idx3 = msg.find("C", 2) + 1;
  size_t idx4 = msg.find("D", 2) + 1;
  size_t idx5 = msg.find("E", 2) + 1;
  size_t idx6 = msg.find("F", 2) + 1;
  joint_positions_deg_[0] = std::stod(msg.substr(idx1, idx2 - idx1));
  joint_positions_deg_[1] = std::stod(msg.substr(idx2, idx3 - idx2));
  joint_positions_deg_[2] = std::stod(msg.substr(idx3, idx4 - idx3));
  joint_positions_deg_[3] = std::stod(msg.substr(idx4, idx5 - idx4));
  joint_positions_deg_[4] = std::stod(msg.substr(idx5, idx6 - idx5));
  joint_positions_deg_[5] = std::stod(msg.substr(idx6));
}

}  // namespace ar_hardware_interface
