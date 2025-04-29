#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
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
    target_angle_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "SMX/TargetImageAngle", 10,
        std::bind(&G1ControlNode::target_angle_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "SMX/Odom", 10,
        std::bind(&G1ControlNode::odom_callback, this, std::placeholders::_1));

    gimbal_state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "SMX/GimbalState", 10,
        std::bind(&G1ControlNode::gimbal_state_callback, this, std::placeholders::_1));

    // ===== 发布 =====
    action_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("SMX/SportCmd", 10);

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

  void gimbal_state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // 提取 yaw_gimbal 和 pitch_gimbal
    if (msg->data.size() >= 6)
    {
      yaw_gimbal = msg->data[2] / 180 * M_PI;  // 提取 frameYaw
      pitch_gimbal = msg->data[1] / 180 * M_PI;  // 提取 framePitch
    }
  }

  void target_angle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
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
    std_msgs::msg::Float64MultiArray out;
    out.data = {static_cast<double>(code),
                static_cast<double>(p1),
                static_cast<double>(p2),
                static_cast<double>(p3),
                static_cast<double>(p4)};
    action_pub_->publish(out);
  }

  //----------------------------------------------------//
  //                  数据成员                          //
  //----------------------------------------------------//
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_angle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     action_pub_;

  // 数据成员：保存从消息中提取的角度信息
  double yaw_robot = 0.0;
  double pitch_robot = 0.0;
  double roll_robot = 0.0;
  double yaw_gimbal = 0.0;
  double pitch_gimbal = 0.0;
  double yaw_image = 0.0;
  double pitch_image = 0.0;

  int robot_posture_enable = 0;
  double robot_posture_yaw = 0;
  double robot_posture_pitch = 0;
  int robot_motion_enable = 0;
  double robot_motion_yaw = 0;

  //----------------------------------------------------//
  //                  目标角度计算与动作发布              //
  //----------------------------------------------------//W
  void update_target_and_publish()
  {
    static auto  last_update_time  = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = current_time - last_update_time;
    last_update_time = current_time;

    publish_action(22202100, yaw_image, pitch_image, 0, 0);

    // 计算目标角度 yaw_target 和 pitch_target
    double yaw_target = yaw_robot + yaw_gimbal + yaw_image;
    double pitch_target = pitch_robot + pitch_gimbal + pitch_image;

    if (std::fabs(yaw_target - yaw_robot) > 1.0)
    {
      robot_motion_enable = 1;
      robot_motion_yaw = (yaw_target - yaw_robot) * 1.0;
      robot_motion_yaw = std::clamp(robot_motion_yaw,  -1.0, 1.0);
      publish_action(25202123, 0.0, 0.0, robot_motion_yaw, 0.0);
    }
    else if (std::fabs(yaw_target - yaw_robot) > 0.5 && robot_motion_enable > 0)
    {
      robot_motion_yaw = (yaw_target - yaw_robot) * 1.0;
      robot_motion_yaw = std::clamp(robot_motion_yaw,  -1.0, 1.0);
      publish_action(25202123, 0.0, 0.0, robot_motion_yaw, 0.0);
    }
    else if (robot_motion_enable > 0)
    {
      robot_motion_enable = 0;
      publish_action(16170000, 0.0, 0.0, 0.0, 0.0);
    }

    if (std::fabs(yaw_target - yaw_robot) > 0.3 || std::fabs(pitch_target - pitch_robot) > 0.3)
      robot_posture_enable = 1;
    if (std::fabs(yaw_target - yaw_robot) < 0.1 && std::fabs(pitch_target - pitch_robot) < 0.1)
      robot_posture_enable = 0;

    if(robot_posture_enable)
    {
      robot_posture_yaw += (yaw_target - yaw_robot) * 0.02;
      robot_posture_pitch += (pitch_target - pitch_robot) * 0.02;
      robot_posture_yaw   = std::clamp(robot_posture_yaw,  -0.5, 0.5);
      robot_posture_pitch = std::clamp(robot_posture_pitch,-0.5, 0.5);
    }

    if (dt.count() >= 0.2)
    {
      robot_posture_yaw = 0;
      robot_posture_pitch = 0;
    }

    publish_action(22232400, robot_posture_yaw, robot_posture_pitch, 0.0, 0.0);

    // // 输出目标角度
    // std::cout << "yaw_robot-" << yaw_robot << " pitch_robot-" << pitch_robot << std::endl;
    // std::cout << "yaw_gimbal-" << yaw_gimbal << " pitch_gimbal-" << pitch_gimbal << std::endl;
    // std::cout << "yaw_image-" << yaw_image << " pitch_image-" << pitch_image << std::endl;
    // std::cout << "yaw_target - yaw_robot: " << yaw_target - yaw_robot << std::endl;
    // std::cout << "robot_motion_yaw: " << robot_motion_yaw << std::endl;
    // std::cout << "________________"  << std::endl;
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
