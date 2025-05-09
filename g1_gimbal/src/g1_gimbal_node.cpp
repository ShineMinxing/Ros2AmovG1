#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "amovGimbal/amov_gimbal.h"
#include "serial/serial.h"
#include <chrono>
#include <sstream>
#include <mutex>

// 基于 serial 库的 IOStream 实现
class UART : public amovGimbal::IOStreamBase
{
private:
  serial::Serial *ser;
public:
  UART(const std::string &port)
  {
    ser = new serial::Serial(
      port, 115200, serial::Timeout::simpleTimeout(500));
  }
  ~UART()
  {
    ser->close(); delete ser;
  }
  bool open() override    { ser->open(); return true; }
  bool close() override   { ser->close(); return true; }
  bool isOpen() override  { return ser->isOpen(); }
  bool isBusy() override  { return false; }
  uint32_t inPutBytes(IN uint8_t *byte) override {
    return ser->read(byte,1) ? 1 : 0;
  }
  uint32_t outPutBytes(IN uint8_t *byte, uint32_t len) override {
    return ser->write(byte, len);
  }
};

class GimbalNode : public rclcpp::Node
{
public:
  explicit GimbalNode(const rclcpp::NodeOptions &options)
  : Node("g1_gimbal_node", options)
  {
    // 声明并获取参数
    auto state_topic = declare_parameter<std::string>(
      "GIMBAL_STATE_TOPIC", "TEST/GimbalState");
    auto cmd_topic   = declare_parameter<std::string>(
      "GIMBAL_CMD_TOPIC",   "TEST/JoyFloatCmd");
    auto port_name   = declare_parameter<std::string>(
      "UART_PORT",         "/dev/ttyUSB0");
    auto gimbal_id   = declare_parameter<std::string>(
      "GIMBAL_ID",         "G1");

    // 初始化 UART + 云台
    uart_   = std::make_shared<UART>(port_name);
    gimbal_ = std::make_shared<amovGimbal::gimbal>(gimbal_id, uart_.get());
    gimbal_->startStack();

    // 发布云台状态
    pub_state_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      state_topic, 10);

    // 注册状态回调
    auto cb = [](double fr, double fp, double fy,
                 double ir, double ip, double iy,
                 double fx, double fy2, void *caller) {
      static_cast<GimbalNode*>(caller)
        ->onGimbalUpdate(fr, fp, fy, ir, ip, iy, fx, fy2);
    };
    gimbal_->parserAuto(cb, this);
    gimbal_->setGimabalHome();

    // 订阅命令话题
    sub_cmd_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      cmd_topic, 10,
      std::bind(&GimbalNode::onTargetAngle, this,
                std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "GimbalNode started: port=%s id=%s\n"
      "  state_topic=%s cmd_topic=%s",
      port_name.c_str(), gimbal_id.c_str(),
      state_topic.c_str(), cmd_topic.c_str());
  }

private:
  void onGimbalUpdate(double fr, double fp, double fy,
                      double ir, double ip, double iy,
                      double fx, double fy2)
  {
    std::lock_guard<std::mutex> lock(mt_);
    frameYaw_  = fy;
    imuPitch_  = ip;
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {fr, fp, fy, ir, ip, iy, fx, fy2};
    pub_state_->publish(msg);
  }

  void onTargetAngle(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3 || msg->data[0] != 30000001) return;
    double angX = msg->data[1], angY = msg->data[2];
    double currentYaw=0, currentPitch=0;
    { std::lock_guard<std::mutex> lock(mt_);
      currentYaw = frameYaw_; currentPitch = imuPitch_; }

    AMOV_GIMBAL_POS_T speed{};
    speed.yaw   = angX;
    speed.pitch = angY;
    speed.roll  = 0.0;
    if (currentPitch > 45)  speed.pitch = 10;
    if (currentPitch < -45) speed.pitch = -10;
    gimbal_->setGimabalSpeed(speed);
  }

  // 成员变量
  std::shared_ptr<UART> uart_;
  std::shared_ptr<amovGimbal::gimbal> gimbal_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cmd_;
  double frameYaw_{0}, imuPitch_{0};
  std::mutex mt_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.arguments({
    "--ros-args",
    "--params-file",
    "/home/unitree/ros2_ws/LeggedRobot/src/Ros2AmovG1/config.yaml"
  });
  auto node = std::make_shared<GimbalNode>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}