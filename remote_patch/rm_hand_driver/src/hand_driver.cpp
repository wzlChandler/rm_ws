#include "rm_hand_driver/hand_driver.hpp"

namespace rm_hand_driver {

HandDriver::HandDriver(ros::NodeHandle& nh, int baudrate, int slave_id, int modbus_port)
    : baudrate_(baudrate),
      slave_id_(slave_id),
      modbus_port_(modbus_port),
      rs485_ready_(false) {
  write_regs_pub_ = nh.advertise<rm_msgs::Write_TCPandRTU>("/rm_driver/Write_Registers", 10);
  rs485_mode_pub_ = nh.advertise<rm_msgs::Set_Modbus_Mode>("/rm_driver/Set_Modbus_Mode", 1);
  rs485_mode_result_sub_ = nh.subscribe(
      "/rm_driver/Set_Modbus_Mode_Result", 1, &HandDriver::toolRs485Result, this);
  write_result_sub_ = nh.subscribe(
      "/rm_driver/Write_Registers_Result", 1, &HandDriver::writeRegistersResult, this);
  rs485_init_timer_ = nh.createTimer(
      ros::Duration(1.0), &HandDriver::initializeToolRs485, this);
}

void HandDriver::initializeToolRs485(const ros::TimerEvent&) {
  if (rs485_ready_) {
    rs485_init_timer_.stop();
    return;
  }
  if (rs485_mode_pub_.getNumSubscribers() == 0) {
    ROS_WARN_THROTTLE(5.0, "waiting for fourth-generation tool RS485 interface");
    return;
  }

  rm_msgs::Set_Modbus_Mode msg;
  msg.port = modbus_port_;
  msg.baudrate = baudrate_;
  msg.timeout = 2;
  rs485_mode_pub_.publish(msg);
  ROS_INFO_THROTTLE(5.0, "configuring tool Modbus: port=%d baudrate=%d timeout=2",
                    modbus_port_, baudrate_);
}

void HandDriver::toolRs485Result(const std_msgs::Bool::ConstPtr& msg) {
  rs485_ready_ = msg->data;
  if (rs485_ready_) {
    rs485_init_timer_.stop();
    ROS_INFO("tool Modbus-RTU mode is ready");
  } else {
    ROS_ERROR("failed to configure tool Modbus-RTU mode");
  }
}

void HandDriver::writeRegistersResult(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    ROS_INFO("hand register write confirmed by controller");
  } else {
    ROS_ERROR("hand register write rejected by controller");
  }
}

std::vector<int> HandDriver::normalizedToHardware(const std::vector<int>& values) const {
  std::vector<int> scaled;
  scaled.reserve(values.size());
  for (int value : values) {
    const int normalized = value < 0 ? 0 : (value > 1000 ? 1000 : value);
    scaled.push_back((normalized * 65535 + 500) / 1000);
  }
  return scaled;
}

std::vector<int> HandDriver::regsToBytes(const std::vector<int>& regs) const {
  std::vector<int> bytes;
  bytes.reserve(regs.size() * 2);
  for (int value : regs) {
    const uint16_t reg = static_cast<uint16_t>(value);
    bytes.push_back((reg >> 8) & 0xFF);
    bytes.push_back(reg & 0xFF);
  }
  return bytes;
}

bool HandDriver::publishRegs(int address, const std::vector<int>& regs) {
  std::lock_guard<std::mutex> lock(mtx_);

  if (!rs485_ready_) {
    ROS_ERROR("cannot write hand registers: tool RS485 mode is not ready");
    return false;
  }
  if (write_regs_pub_.getNumSubscribers() == 0) {
    ROS_ERROR("cannot write hand registers: /rm_driver/Write_Registers is not connected");
    return false;
  }

  rm_msgs::Write_TCPandRTU msg;
  msg.address = address;
  msg.data = regsToBytes(regs);
  msg.port = modbus_port_;
  msg.type = 1;
  msg.device = slave_id_;

  write_regs_pub_.publish(msg);
  return true;
}

bool HandDriver::writeRawRegs(int address, const std::vector<int>& regs) {
  return publishRegs(address, regs);
}

bool HandDriver::writeSingleRegister(int address, int value) {
  return publishRegs(address, std::vector<int>{value});
}

bool HandDriver::writePythonStyle(const std::vector<int>& payload6) {
  if (payload6.size() != 6) return false;
  const std::vector<int> hardware = normalizedToHardware(payload6);
  ROS_INFO("normalized hand target -> hardware: [%d, %d, %d, %d, %d, %d]",
           hardware[0], hardware[1], hardware[2], hardware[3], hardware[4], hardware[5]);
  return publishRegs(1135, hardware);
}

bool HandDriver::openHand() {
  return publishRegs(1135, std::vector<int>(6, 0));
}

bool HandDriver::closeHand(uint16_t value) {
  return writePythonStyle({value, value, value, value, value, 0});
}

}  // namespace rm_hand_driver
