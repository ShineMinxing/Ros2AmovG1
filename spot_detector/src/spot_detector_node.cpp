#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SpotDetectorNode : public rclcpp::Node
{
public:
  SpotDetectorNode()
  : Node("spot_detector_node")
  {
    // 订阅图像话题
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "SMX/Gimbal_Camera", 10,
      std::bind(&SpotDetectorNode::imageCallback, this, std::placeholders::_1));

    // 发布标记后的视频图像
    video_pub_ = this->create_publisher<sensor_msgs::msg::Image>("SMX/Target_Video", 10);

    // 发布角度信息，数据为 [angle_x_deg, angle_y_deg, tilt_deg]
    angle_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("SMX/Target_Angle", 10);

    // 设置相机的水平和垂直视场（单位：度）
    fov_h_ = 125.0;
    fov_v_ = 69.0;

    RCLCPP_INFO(this->get_logger(), "SpotDetectorNode 启动成功。");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // 将 ROS 图像消息转换为 OpenCV 格式（BGR）
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
      return;
    }
    cv::Mat frame = cv_ptr->image;
    int width = frame.cols;
    int height = frame.rows;

    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "收到空帧！");
      return;
    }

    // 检测蓝光点：要求 B > 150 且 B 大于 R 和 G
    bool found = false;
    int max_b_value = -1;
    int max_b_x = -1;
    int max_b_y = -1;

    for (int y = 0; y < height; y++) {
      const uchar* row_ptr = frame.ptr<uchar>(y);
      for (int x = 0; x < width; x++) {
        int b = row_ptr[x * 3 + 0];
        int g = row_ptr[x * 3 + 1];
        int r = row_ptr[x * 3 + 2];

        // 满足条件：B 大于 150 且 B > R 且 B > G
        if (b > 150 && b > (r+80) && b > (g+80)) {
          if (b > max_b_value) {
            max_b_value = b;
            max_b_x = x;
            max_b_y = y;
            found = true;
          }
        }
      }
    }

    // 如果检测到蓝光点，则计算角度；否则角度均返回 0
    double angle_x_deg = 0.0, angle_y_deg = 0.0, tilt_deg = 0.0;
    if (found) {
      // 在图像上画一个绿色圆圈标记该点
      cv::circle(frame, cv::Point(max_b_x, max_b_y), 10, cv::Scalar(0, 255, 0), 2);

      // 计算该点相对于图像中心的偏移
      double dx = static_cast<double>(max_b_x) - (width / 2.0);
      double dy = static_cast<double>(max_b_y) - (height / 2.0);
      double ratio_x = dx / (width / 2.0);   // 范围 -1 ~ +1
      double ratio_y = dy / (height / 2.0);    // 范围 -1 ~ +1

      // 根据视场角计算角度（假设图像中心为 0°）
      angle_x_deg = ratio_x * (fov_h_ / 2.0);
      angle_y_deg = ratio_y * (fov_v_ / 2.0);
      tilt_deg = 0.0;  // 此处未计算 tilt，可根据需要扩展

    }

    // 将修改后的图像发布到 SMX/Target_Video
    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    video_pub_->publish(*out_msg);

    // 组装角度信息并发布到 SMX/Target_Angle (Float32MultiArray)
    std_msgs::msg::Float32MultiArray angle_msg;
    angle_msg.data.push_back(static_cast<float>(angle_x_deg));
    angle_msg.data.push_back(static_cast<float>(angle_y_deg));
    angle_msg.data.push_back(static_cast<float>(tilt_deg));
    angle_pub_->publish(angle_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr video_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_pub_;
  double fov_h_;
  double fov_v_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpotDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
