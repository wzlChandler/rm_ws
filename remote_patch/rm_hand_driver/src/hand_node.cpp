#include "rm_hand_driver/hand_driver.hpp"

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

using rm_hand_driver::HandDriver;

class HandNode {
public:
  HandNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh),
        pnh_(pnh),
        driver_(
            nh_,
            pnh.param<int>("baudrate", 115200),
            pnh.param<int>("slave_id", 2),
            pnh.param<int>("modbus_port", 1)),
        close_value_(pnh.param<int>("close_value", 500)) {
    loadDefaultGestures();

    // 这里只是桥接到 rm_driver，不再自己 connect()
    ROS_INFO("hand node works in bridge mode via /rm_driver/Write_Registers");

    target_sub_ = nh_.subscribe("/hand/target", 1, &HandNode::targetCb, this);
    python_style_sub_ = nh_.subscribe("/hand/python_style", 1, &HandNode::pythonStyleCb, this);
    gesture_sub_ = nh_.subscribe("/hand/gesture", 1, &HandNode::gestureCb, this);

    raw_single_sub_ = nh_.subscribe("/hand/raw_single", 1, &HandNode::rawSingleCb, this);
    raw_regs_sub_ = nh_.subscribe("/hand/raw_regs", 1, &HandNode::rawRegsCb, this);

    open_srv_  = nh_.advertiseService("/hand/open",  &HandNode::openCb, this);
    close_srv_ = nh_.advertiseService("/hand/close", &HandNode::closeCb, this);

    ROS_INFO("hand node ready");
    ROS_INFO("/hand/gesture      : String");
    ROS_INFO("/hand/python_style : Int32MultiArray [6 actuator values]");
    ROS_INFO("/hand/target       : UInt16MultiArray [r0 r1 r2 r3 r4 r5]");
    ROS_INFO("/hand/raw_single   : Int32MultiArray [address value]");
    ROS_INFO("/hand/raw_regs     : Int32MultiArray [address reg0 reg1 ...]");
    ROS_INFO("service /hand/open");
    ROS_INFO("service /hand/close");

    logAvailableGestures();
  }

private:
  static std::string normalize(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
  }

  void loadDefaultGestures() {
    gestures_["open"]     = {0, 0, 0, 0, 0, 0};
    gestures_["close"]    = {1000, 1000, 1000, 1000, 1000, 0};

    // 两指捏取：拇指 + 食指
    gestures_["pinch_2"]  = {800, 800, 0, 0, 0, 0};

    // 三指捏取：拇指 + 食指 + 中指
    gestures_["pinch_3"]  = {800, 800, 800, 0, 0, 0};

    // 圆柱包裹抓
    gestures_["cylinder"] = {700, 900, 900, 900, 900, 0};

    // 常用动作
    gestures_["ok"]       = {700, 700, 0, 0, 0, 0};
    gestures_["thumb"]    = {0, 1000, 1000, 1000, 1000, 0};
    gestures_["number_1"] = {1000, 0, 1000, 1000, 1000, 0};
    gestures_["number_2"] = {1000, 0, 0, 1000, 1000, 0};
    gestures_["number_5"] = {0, 0, 0, 0, 0, 0};
  }

  void logAvailableGestures() const {
    std::string names;
    for (auto it = gestures_.begin(); it != gestures_.end(); ++it) {
      if (!names.empty()) names += ", ";
      names += it->first;
    }
    ROS_INFO_STREAM("available gestures: " << names);
  }

  bool executeGesture(const std::string& gesture_name) {
    const std::string key = normalize(gesture_name);
    auto it = gestures_.find(key);
    if (it == gestures_.end()) {
      ROS_WARN_STREAM("unknown gesture: " << gesture_name);
      return false;
    }

    ROS_INFO_STREAM("execute gesture: " << key);
    return driver_.writePythonStyle(it->second);
  }

  void targetCb(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 6) {
      ROS_WARN("/hand/target expects 6 values");
      return;
    }

    std::vector<int> regs(6, 0);
    for (size_t i = 0; i < 6; ++i) {
      regs[i] = static_cast<int>(msg->data[i]);
    }

    ROS_INFO("target regs: [%d, %d, %d, %d, %d, %d]",
             regs[0], regs[1], regs[2], regs[3], regs[4], regs[5]);

    bool ok = driver_.writePythonStyle(regs);
    ROS_INFO("target write result: %s", ok ? "true" : "false");
  }

  void pythonStyleCb(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 6) {
      ROS_WARN("/hand/python_style expects 6 values");
      return;
    }

    std::vector<int> payload;
    payload.reserve(6);
    for (auto v : msg->data) {
      payload.push_back(v);
    }

    ROS_INFO("python_style payload size=%zu", payload.size());
    bool ok = driver_.writePythonStyle(payload);
    ROS_INFO("python_style write result: %s", ok ? "true" : "false");
  }

  void gestureCb(const std_msgs::String::ConstPtr& msg) {
    bool ok = executeGesture(msg->data);
    ROS_INFO("gesture write result: %s", ok ? "true" : "false");
  }

  void rawSingleCb(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 2) {
      ROS_WARN("/hand/raw_single format: [address value]");
      return;
    }

    int address = msg->data[0];
    int value = msg->data[1];

    ROS_INFO("raw single: addr=%d value=%d", address, value);
    bool ok = driver_.writeSingleRegister(address, value);
    ROS_INFO("raw single result: %s", ok ? "true" : "false");
  }

  void rawRegsCb(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 2) {
      ROS_WARN("/hand/raw_regs format: [address reg0 reg1 ...]");
      return;
    }

    int address = msg->data[0];
    std::vector<int> regs;
    regs.reserve(msg->data.size() - 1);

    for (size_t i = 1; i < msg->data.size(); ++i) {
      regs.push_back(msg->data[i]);
    }

    ROS_INFO("raw regs: addr=%d count=%zu", address, regs.size());
    bool ok = driver_.writeRawRegs(address, regs);
    ROS_INFO("raw regs result: %s", ok ? "true" : "false");
  }

  bool openCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    res.success = driver_.openHand();
    res.message = res.success ? "opened" : "open failed";
    ROS_INFO("openHand result: %s", res.success ? "true" : "false");
    return true;
  }

  bool closeCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    res.success = driver_.closeHand(static_cast<uint16_t>(close_value_));
    res.message = res.success ? "closed" : "close failed";
    ROS_INFO("closeHand result: %s", res.success ? "true" : "false");
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  HandDriver driver_;
  int close_value_;

  std::map<std::string, std::vector<int>> gestures_;

  ros::Subscriber target_sub_;
  ros::Subscriber python_style_sub_;
  ros::Subscriber gesture_sub_;
  ros::Subscriber raw_single_sub_;
  ros::Subscriber raw_regs_sub_;

  ros::ServiceServer open_srv_;
  ros::ServiceServer close_srv_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm_hand_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  HandNode node(nh, pnh);
  ros::spin();
  return 0;
}
