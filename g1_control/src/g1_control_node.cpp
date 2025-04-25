#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"  // 用于Odometry消息
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // 用于tf2四元数到欧拉角转换
#include <array>
#include <chrono>
#include <cmath>
#include <iostream> 

class G1ControlNode : public rclcpp::Node
{
public:
  G1ControlNode() : Node("g1_control_node")
  {
    // ===== 订阅 =====
    target_angle_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "SMX/TargetImageAngle", 10,
        std::bind(&G1ControlNode::target_angle_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "SMX/Odom", 10,
        std::bind(&G1ControlNode::odom_callback, this, std::placeholders::_1));

    gimbal_state_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "SMX/GimbalState", 10,
        std::bind(&G1ControlNode::gimbal_state_callback, this, std::placeholders::_1));

    // ===== 发布 =====
    action_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("SMX/SportCmd", 10);

    RCLCPP_INFO(get_logger(), "g1_control_node started.");
  }

private:
  //----------------------------------------------------//
  //                    回调函数                        //
  //----------------------------------------------------//

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 提取 yaw_robot 和 pitch_robot（通过四元数转换为欧拉角）
    tf2::Quaternion quat(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll_robot, pitch_robot, yaw_robot);  // 获取 roll, pitch, yaw

    pitch_robot = - pitch_robot;
  }

  void gimbal_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // 提取 yaw_gimbal 和 pitch_gimbal
    if (msg->data.size() >= 6)
    {
      yaw_gimbal = msg->data[2] / 180 * M_PI;  // 提取 frameYaw
      pitch_gimbal = msg->data[1] / 180 * M_PI;  // 提取 framePitch
    }
  }

  void target_angle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 2)
    {
      yaw_image = -msg->data[0] / 180 * M_PI;  // 提取 yaw_image
      pitch_image = msg->data[1] / 180 * M_PI;  // 提取 pitch_image
      update_target_and_publish();
    }
  }

  //----------------------------------------------------//
  //                动作发布统一接口                    //
  //----------------------------------------------------//
  void publish_action(double code, double p1, double p2, double p3, double p4)
  {
    std_msgs::msg::Float32MultiArray out;
    out.data = {static_cast<float>(code),
                static_cast<float>(p1),
                static_cast<float>(p2),
                static_cast<float>(p3),
                static_cast<float>(p4)};
    action_pub_->publish(out);
  }

  //----------------------------------------------------//
  //                  数据成员                          //
  //----------------------------------------------------//
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_angle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gimbal_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr     action_pub_;

  // 数据成员：保存从消息中提取的角度信息
  double yaw_robot = 0.0;
  double pitch_robot = 0.0;
  double roll_robot = 0.0; // Roll 角度
  double yaw_gimbal = 0.0;
  double pitch_gimbal = 0.0;
  double yaw_image = 0.0;
  double pitch_image = 0.0;

  //----------------------------------------------------//
  //                  目标角度计算与动作发布              //
  //----------------------------------------------------//
  void update_target_and_publish()
  {
    // 计算目标角度 yaw_target 和 pitch_target
    double yaw_target = yaw_robot + yaw_gimbal + yaw_image;
    double pitch_target = pitch_robot + pitch_gimbal + pitch_image;

    // 输出目标角度
    std::cout << "yaw_robot: " << yaw_robot << ", pitch_robot: " << pitch_robot << std::endl;
    std::cout << "yaw_gimbal: " << yaw_gimbal << ", pitch_gimbal: " << pitch_gimbal << std::endl;
    std::cout << "yaw_image: " << yaw_image << ", pitch_image: " << pitch_image << std::endl;
    std::cout << "yaw_target: " << yaw_target << ", pitch_target: " << pitch_target << std::endl;
    std::cout << "________________"  << std::endl;

    publish_action(22202100, yaw_image, pitch_image, 0, 0);

    // // 判断是否需要发布动作
    // if (std::fabs(yaw_target - yaw_robot) > 0.9)
    // {
    //   publish_action(25202123, 0.0, 0.0, yaw_target - yaw_robot, 0.0);
    // }

    // if (std::fabs(yaw_target - yaw_robot) > 0.3 || std::fabs(pitch_target - pitch_robot) > 0.3)
    // {
    //   publish_action(22232400, accumulated_yaw, accumulated_pitch, 0, 0);
    // }
  }
};

// =========================== main =============================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1ControlNode>());
  rclcpp::shutdown();
  return 0;
}
