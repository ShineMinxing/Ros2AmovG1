#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class VideoPlayNode : public rclcpp::Node
{
public:
  VideoPlayNode() : Node("video_play_node")
  {
    // 创建发布器，发布到 SMXFE/Gimbal_Camera 话题
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("SMXFE/Gimbal_Camera", 10);

    // 视频文件路径（请确保文件存在且权限正确）
    std::string video_path = "/home/smx/NetworkShare/Video1_4.mp4";
    cap_.open(video_path, cv::CAP_ANY);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开视频文件: %s", video_path.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "成功打开视频: %s", video_path.c_str());
    }

    // 定时器：每33毫秒发布一帧（约30Hz）
    timer_ = this->create_wall_timer(33ms, std::bind(&VideoPlayNode::timerCallback, this));
  }

private:
  void timerCallback()
  {
    if (!cap_.isOpened()) {
      return;
    }

    cv::Mat frame;
    cap_ >> frame; // 读取一帧
    if (frame.empty()) {
      // 如果视频读到末尾，循环回到起始位置
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      cap_ >> frame;
      if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "读取视频帧为空！");
        return;
      }
    }

    // 将OpenCV图像转换为ROS图像消息
    auto cv_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame);
    cv_img.header.stamp = this->now();
    cv_img.header.frame_id = "video_frame";
    publisher_->publish(*cv_img.toImageMsg());
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VideoPlayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
