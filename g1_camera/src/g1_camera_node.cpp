#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

class GimbalCameraNode : public rclcpp::Node
{
public:
  GimbalCameraNode() : Node("g1_camera_node")
  {
    // 发布话题名可和原先的 SMX/Gimbal_Camera 一致，方便兼容
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("SMX/Gimbal_Camera", 10);

    // 构建GStreamer管线
    std::string pipeline =
        "rtspsrc location=rtsp://192.168.123.64:554/H264 "
        "protocols=GST_RTSP_LOWER_TRANS_UDP latency=0 "
        "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink";

    cap_.open(pipeline, cv::CAP_GSTREAMER);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer RTSP pipeline!");
    } else {
      RCLCPP_INFO(this->get_logger(), "RTSP stream opened successfully (low-latency).");
    }

    // 定时器：约33ms发布一帧图像 -> ~30fps
    auto period = std::chrono::milliseconds(33);
    timer_ = this->create_wall_timer(
      period, std::bind(&GimbalCameraNode::timerCallback, this));
  }

private:
  void timerCallback()
  {
    if (!cap_.isOpened()) {
      return;
    }

    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
      return;
    }

    // 将OpenCV图像包装成ROS消息并发布
    auto cv_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame);
    cv_img.header.stamp = this->now();
    cv_img.header.frame_id = "camera_link";
    image_pub_->publish(*cv_img.toImageMsg());
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GimbalCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
