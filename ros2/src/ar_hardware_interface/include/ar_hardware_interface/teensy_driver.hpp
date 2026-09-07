#ifndef TEENSY_DRIVER_H
#define TEENSY_DRIVER_H

#include <boost/asio.hpp>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "math.h"
#include "time.h"

namespace ar_hardware_interface {

class TeensyDriver {
 public:
  /// Default receive timeout for normal serial exchanges (milliseconds)
  static constexpr int DEFAULT_RECEIVE_TIMEOUT_MS = 5000;
  /// Receive timeout during calibration (milliseconds, 10 minutes)
  static constexpr int CALIBRATION_TIMEOUT_MS = 600000;

  void init(std::string port, int baudrate, int num_joints);
  void setStepperSpeed(std::vector<double>& max_speed,
                       std::vector<double>& max_accel);
  void update(std::vector<double>& pos_commands,
              std::vector<double>& joint_states);
  void getJointPositions(std::vector<double>& joint_positions);
  void calibrateJoints();
  void toggleClosedLoop();
  void setClosedLoopDesired(bool enable);
  bool isClosedLoopEnabled() const { return closed_loop_enabled_; }
  bool isConnected() const { return connected_; }

  TeensyDriver();

 private:
  bool initialised_ = false;
  bool connected_ = false;
  std::string version_;
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
  int num_joints_;
  std::vector<double> joint_positions_deg_;
  std::vector<int> enc_calibrations_;
  rclcpp::Logger logger_ = rclcpp::get_logger("teensy_driver");
  rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);

  // Serial port mutex for thread safety
  std::mutex serial_mutex_;
  bool closed_loop_enabled_ = false;  // mirrors firmware CLOSED_LOOP_ENABLE

  // Comms with teensy
  void exchange(std::string outMsg);  // exchange joint commands/state
  void exchangeWithTimeout(std::string outMsg,
                           int receive_timeout_ms);  // exchange with timeout
  bool transmit(std::string outMsg, std::string& err);
  bool receive(std::string& inMsg, int timeout_ms);  // read with timeout (ms)
  void sendCommand(std::string outMsg);              // send arbitrary commands

  void checkInit(std::string msg);
  void updateEncoderCalibrations(std::string msg);
  void updateJointPositions(std::string msg);

  /// Drain stale data from the OS serial buffer
  void drainBuffer();
};

}  // namespace ar_hardware_interface

#endif  // TEENSY_DRIVER
