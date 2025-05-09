#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

class GimbalCameraNode : public rclcpp::Node
{
public:
  explicit GimbalCameraNode(const rclcpp::NodeOptions & options)
  : Node("g1_camera_node", options)
  {
    // 参数声明和获取
    auto topic_name = this->declare_parameter<std::string>(
      "GIMBAL_CAMERA", "TEST/GimbalCamera");
    auto pipeline = this->declare_parameter<std::string>(
      "GIMBAL_GSTREAMER",
      "rtspsrc location=rtsp://192.168.123.64:554/H264 "
      "protocols=GST_RTSP_LOWER_TRANS_UDP latency=50 drop-on-latency=true "
      "! rtph264depay "
      "! h264parse "
      "! nvv4l2decoder enable-max-performance=1 disable-dpb=1 "
      "! nvvidconv output-buffers=1 "
      "! video/x-raw,format=(string)BGRx "
      "! videoconvert ! video/x-raw,format=BGR "
      "! appsink sync=false max-buffers=1 drop=true"
    );

    // 创建 Publisher
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      topic_name, 10);

    // 打开 GStreamer RTSP 管线
    cap_.open(pipeline, cv::CAP_GSTREAMER);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to open GStreamer pipeline: %s", pipeline.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
        "RTSP stream opened on topic %s", topic_name.c_str());
    }

    // 定时器：每 33ms 发布一帧
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&GimbalCameraNode::timerCallback, this)
    );
  }

private:
  void timerCallback()
  {
    if (!cap_.isOpened()) return;
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) return;

    auto img_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    img_msg->header.stamp = this->now();
    img_msg->header.frame_id = "camera_link";
    image_pub_->publish(*img_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 构造 NodeOptions 并自动加载同目录下 config.yaml
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args",
    "--params-file",
    "/home/unitree/ros2_ws/LeggedRobot/src/Ros2AmovG1/config.yaml"
  });
  auto node = std::make_shared<GimbalCameraNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}