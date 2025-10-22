#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2, time, os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.last_save_time = 0.0
        self.save_interval = 9.0  # каждые 9 секунд
        self.frame_id = 0
        self.current_pose = None

        self.image_topic = '/world/baylands/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image'
        self.pose_topic = '/odom'  # можно поменять при необходимости

        # Папка для кадров
        self.output_dir = os.path.expanduser('~/images')
        os.makedirs(self.output_dir, exist_ok=True)

        self.pose_file = os.path.join(self.output_dir, 'poses.txt')

        # Подписки
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, self.pose_topic, self.pose_callback, 10)

        self.get_logger().info('ImageSaver node started')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def image_callback(self, msg):
        now = time.time()
        if now - self.last_save_time >= self.save_interval:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = os.path.join(self.output_dir, f"frame_{self.frame_id:04d}.jpg")
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved: {filename}")

            if self.current_pose:
                with open(self.pose_file, "a") as f:
                    p = self.current_pose.position
                    f.write(f"{self.frame_id},{p.x},{p.y},{p.z}\n")

            self.frame_id += 1
            self.last_save_time = now

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
