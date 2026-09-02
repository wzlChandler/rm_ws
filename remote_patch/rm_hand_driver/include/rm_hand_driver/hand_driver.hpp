#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include <ros/ros.h>
#include <rm_msgs/Set_Modbus_Mode.h>
#include <rm_msgs/Write_TCPandRTU.h>
#include <std_msgs/Bool.h>

namespace rm_hand_driver {

class HandDriver {
public:
  HandDriver(ros::NodeHandle& nh, int baudrate, int slave_id, int modbus_port);

  bool writePythonStyle(const std::vector<int>& payload6);
  bool openHand();
  bool closeHand(uint16_t value = 500);

  bool writeSingleRegister(int address, int value);
  bool writeRawRegs(int address, const std::vector<int>& regs);

private:
  void initializeToolRs485(const ros::TimerEvent& event);
  void toolRs485Result(const std_msgs::Bool::ConstPtr& msg);
  void writeRegistersResult(const std_msgs::Bool::ConstPtr& msg);
  bool publishRegs(int address, const std::vector<int>& regs);
  std::vector<int> normalizedToHardware(const std::vector<int>& values) const;
  std::vector<int> regsToBytes(const std::vector<int>& regs) const;

private:
  int baudrate_;
  int slave_id_;
  int modbus_port_;

  ros::Publisher write_regs_pub_;
  ros::Publisher rs485_mode_pub_;
  ros::Subscriber rs485_mode_result_sub_;
  ros::Subscriber write_result_sub_;
  ros::Timer rs485_init_timer_;
  bool rs485_ready_;
  std::mutex mtx_;
};

}  // namespace rm_hand_driver
