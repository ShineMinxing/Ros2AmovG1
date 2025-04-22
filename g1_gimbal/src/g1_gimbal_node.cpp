#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>  // 订阅 SMX/Target_Angle 的消息类型
#include "amovGimbal/amov_gimbal.h"
#include "serial/serial.h"
#include <chrono>
#include <sstream>
#include <mutex>

// Implementation of the IOStreamBase interface class based on Serial library
class UART : public amovGimbal::IOStreamBase
{
private:
  serial::Serial *ser;

public:
  virtual bool open()
  {
    ser->open();
    return true;
  }
  virtual bool close()
  {
    ser->close();
    return true;
  }
  virtual bool isOpen()
  {
    return ser->isOpen();
  }
  virtual bool isBusy()
  {
    return false;
  }
  virtual uint32_t inPutBytes(IN uint8_t *byte)
  {
    if (ser->read(byte, 1)) {
      return 1;
    }
    return 0;
  }
  virtual uint32_t outPutBytes(IN uint8_t *byte, uint32_t length)
  {
    return ser->write(byte, length);
  }

  UART(const std::string &port)
  {
    // 设置115200波特率 & 500ms 超时
    ser = new serial::Serial(port, 115200, serial::Timeout::simpleTimeout(500));
  }
  ~UART()
  {
    ser->close();
    delete ser;
  }
};

class GimbalNode : public rclcpp::Node
{
public:
  GimbalNode()
    : Node("g1_gimbal_node")
    , frameYaw_(0.0)
    , imuPitch_(0.0)
  {
    // 1) 初始化云台串口与 ID
    std::string port = "/dev/ttyUSB0";
    std::string gimbal_id = "G1";
    uart_ = std::make_shared<UART>(port);
    gimbal_ = std::make_shared<amovGimbal::gimbal>(gimbal_id, uart_.get());
    gimbal_->startStack();

    // 2) 发布云台状态到 SMX/Gimbal_State（原先代码保留）
    pub_state_ = this->create_publisher<std_msgs::msg::String>("SMX/Gimbal_State", 10);

    // 3) 注册云台状态回调，获取 frameYaw, imuPitch 等
    auto cb = [](double fr, double fp, double fy,
                 double ir, double ip, double iy,
                 double fx, double fy2, void *caller)
    {
      GimbalNode *node = static_cast<GimbalNode*>(caller);
      if (node) {
        node->onGimbalUpdate(fr, fp, fy, ir, ip, iy, fx, fy2);
      }
    };
    gimbal_->parserAuto(cb, this);
    gimbal_->setGimabalHome();

    // 4) 订阅 SMX/Target_Angle（假设消息中包含 [angleX, angleY, tilt]）
    sub_angle_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "SMX/Target_Angle", 10,
      std::bind(&GimbalNode::onTargetAngle, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "GimbalNode started. port=%s, id=%s",
                port.c_str(), gimbal_id.c_str());

    // [重要] 取消timerCallback()，不再使用周期性动作
    // 原timer_不再定义
  }

private:
  // --------------------------------------------------------------------------------
  // (A) 云台状态回调：只做输出，并保存 frameYaw, imuPitch 以供控制用
  // --------------------------------------------------------------------------------
  void onGimbalUpdate(double frameRoll, double framePitch, double frameYaw,
                      double imuRoll,   double imuPitch,   double imuYaw,
                      double fovX,      double fovY)
  {
    // 保存到成员变量（加锁以防多线程竞争）
    {
      std::lock_guard<std::mutex> lock(gimbal_mutex_);
      frameYaw_  = frameYaw;   // "frameYaw" from the callback
      imuPitch_  = imuPitch;   // "imuPitch" from the callback
    }

    // 发布状态到 SMX/Gimbal_State
    std_msgs::msg::String msg;
    std::stringstream ss;
    ss << "[Gimbal State]\n"
       << "frameYaw  : " << frameYaw   << "\n"
       << "frameRoll : " << frameRoll  << "\n"
       << "framePitch: " << framePitch << "\n"
       << "imuYaw    : " << imuYaw     << "\n"
       << "imuRoll   : " << imuRoll    << "\n"
       << "imuPitch  : " << imuPitch   << "\n"
       << "fovX      : " << fovX       << "\n"
       << "fovY      : " << fovY;
    msg.data = ss.str();
    pub_state_->publish(msg);
  }

  // --------------------------------------------------------------------------------
  // (B) 订阅 SMX/Target_Angle 的回调
  //     - 读取 [angleX, angleY, tilt] (若仅前2个有效，随你定义)
  //     - 结合当前 frameYaw, imuPitch 计算新的云台目标角度
  //     - 调用 setGimabalPos() 进行控制输出
  // --------------------------------------------------------------------------------
  void onTargetAngle(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Target_Angle message size < 2, ignoring...");
      return;
    }

    float angleX = msg->data[0]; // e.g. unit=deg
    float angleY = msg->data[1]; // e.g. unit=deg
    float tilt   = 0.0f;
    if (msg->data.size() >= 3) {
      tilt = msg->data[2];
    }

    // 取出当前 frameYaw, imuPitch
    double currentYaw = 0.0, currentPitch = 0.0;
    {
      std::lock_guard<std::mutex> lock(gimbal_mutex_);
      currentYaw   = frameYaw_;
      currentPitch = imuPitch_;
    }

    AMOV_GIMBAL_POS_T speed;
    speed.yaw   = 3*angleX;
    speed.pitch = 3*angleY;
    speed.roll  = 0.0f;
    
    if (currentPitch > 45)
      speed.pitch = 10;
    if (currentPitch < -45)
      speed.pitch = -10;

    gimbal_->setGimabalSpeed(speed);
  }

  // --------------------------------------------------------------------------------
  // 成员变量
  // --------------------------------------------------------------------------------
  std::shared_ptr<UART> uart_;
  std::shared_ptr<amovGimbal::gimbal> gimbal_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_angle_;

  // 由 onGimbalUpdate() 更新的云台状态
  double frameYaw_;
  double imuPitch_;
  std::mutex gimbal_mutex_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GimbalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
