#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
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
      "GIMBAL_CAMERA", "NoYamlRead/GimbalCamera");
    std::string gst_pipeline;
    this->get_parameter_or<std::string>(
      "GIMBAL_GSTREAMER", gst_pipeline,
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

    pub_camera_raw_enable_ = this->declare_parameter<bool>("pub_camera_raw_enable", false);
    pub_camera_compressed_enable_ = this->declare_parameter<bool>("pub_camera_compressed_enable", false);

    // 创建 Publisher    
    if (pub_camera_compressed_enable_) {
      pub_camera_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_name + "_Compressed", 10);
    }
    if (pub_camera_raw_enable_) {
      pub_camera_raw_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_name + "_Raw", 10);
    }

    // 定时器：每 33ms 发布一帧
    if (pub_camera_raw_enable_ || pub_camera_compressed_enable_) {
      cap_.open(gst_pipeline, cv::CAP_GSTREAMER);
      if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开视频流");
        return;
      }
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&GimbalCameraNode::timerCallback, this)
      );
    }
  }

private:
  void timerCallback()
  {
    cv::Mat frame;
    if (cap_.read(frame)) {
      if (pub_camera_raw_enable_) {
        auto msg = sensor_msgs::msg::Image();
        msg.header = msg.header;
        msg.height = frame.rows;
        msg.width  = frame.cols;
        msg.encoding     = "bgr8";
        msg.is_bigendian = false;
        msg.step         = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        msg.data.assign(frame.datastart, frame.dataend);
        pub_camera_raw_->publish(msg);
      }

      if (pub_camera_compressed_enable_) {
        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf);
        sensor_msgs::msg::CompressedImage cim;
        cim.header.stamp    = this->get_clock()->now();
        cim.header.frame_id = "camera";
        cim.format = "jpeg";
        cim.data   = std::move(buf);
        pub_camera_compressed_->publish(cim);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "采集视频帧失败");
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_raw_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_camera_compressed_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool pub_camera_raw_enable_;
  bool pub_camera_compressed_enable_;
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