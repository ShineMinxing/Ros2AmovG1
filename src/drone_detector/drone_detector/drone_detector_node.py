import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import torch
from torchvision import models, transforms
import math
import cv2
from cv_bridge import CvBridge
from PIL import Image as PILImage
from ament_index_python.packages import get_package_share_directory

class DroneInferenceNode(Node):
    def __init__(self):
        super().__init__('drone_detector_node')

        # 订阅原始图像话题
        self.image_sub_ = self.create_subscription(
            Image,
            'SMXFE/Gimbal_Camera',  # 原始图像话题
            self.image_callback,
            10
        )

        # 发布无人机角度的Float32MultiArray（仅包含x, y, tilt）
        self.angle_pub_ = self.create_publisher(Float32MultiArray, 'SMXFE/Target_Angle', 10)

        # 发布带有绿色标记的视频图像
        self.video_pub_ = self.create_publisher(Image, 'SMXFE/Target_Video', 10)

        self.bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'使用设备: {self.device}')

        # 通过 ROS2 share 目录获取模型文件路径
        share_dir = get_package_share_directory('drone_detector')
        model_path = os.path.join(share_dir, 'resource', 'drone_model_best_0.pth')
        self.model = self.load_model(model_path)

        # 与训练时一致的图像预处理
        self.data_transforms = transforms.Compose([
            transforms.Resize((256, 256)),
            transforms.ToTensor(),
        ])

        # 相机参数：水平FOV和垂直FOV，单位为度
        self.fov_h = 125.0  # 水平视场
        self.fov_v = 69.0   # 垂直视场

    def load_model(self, model_path):
        model = models.resnet18()
        num_ftrs = model.fc.in_features
        # 模型输出3个通道：x, y, tilt
        model.fc = torch.nn.Linear(num_ftrs, 3)
        model.load_state_dict(torch.load(model_path, map_location=self.device))
        model = model.to(self.device)
        model.eval()
        self.get_logger().info("模型加载完成！")
        return model

    def image_callback(self, msg):
        # 将ROS图像消息转换为OpenCV格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape

        # 预处理：转换为RGB并用PIL包装
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(cv_image_rgb)
        input_image = self.data_transforms(pil_image).unsqueeze(0).to(self.device)

        # 模型推理
        with torch.no_grad():
            outputs = self.model(input_image)
        x_norm, y_norm, tilt_norm = outputs[0].tolist()

        # 将归一化后的输出反映射回原图像像素坐标
        scale_x = width / 256.0
        scale_y = height / 256.0
        self.get_logger().info(f"x_n={x_norm:.4f}, y_n={y_norm:.4f}, s_x={scale_x:.4f}, s_y={scale_y:.4f}")
        x_orig = x_norm * scale_x
        y_orig = y_norm * scale_y
        # tilt转换为弧度
        tilt = tilt_norm / 180.0 * math.pi

        # 在原图上画一个绿色圆圈标记无人机位置
        center_pt = (int(round(x_orig)), int(round(y_orig)))
        cv2.circle(cv_image, center_pt, 10, (0, 255, 0), 2)

        # 发布标记后的视频图像
        video_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.video_pub_.publish(video_msg)

        # 将像素坐标转换为角度：以图像中心为0°（水平和垂直）
        dx = x_orig - (width / 2.0)
        dy = y_orig - (height / 2.0)
        ratio_x = dx / (width / 2.0)    # 范围 -1~+1
        ratio_y = dy / (height / 2.0)   # 范围 -1~+1

        # 将比例转换为角度（左右边缘分别为 -fov_h/2 ~ +fov_h/2；上下边缘为 -fov_v/2 ~ +fov_v/2）
        angle_x_deg = ratio_x * (self.fov_h / 2.0)
        angle_y_deg = ratio_y * (self.fov_v / 2.0)
        # 将 tilt 也转换为度数
        tilt_deg = tilt * (180.0 / math.pi)

        # 组装角度信息并发布：依次为水平角、垂直角、倾角
        angle_msg = Float32MultiArray()
        angle_msg.data = [angle_x_deg, angle_y_deg, tilt_deg]
        self.angle_pub_.publish(angle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
