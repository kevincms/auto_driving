#!/usr/bin/env python3

import cv2, time, os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray

import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MainNode(Node):
    def __init__(self, folder_path):
        super().__init__('main_node')
        # camera 사전준비
        self.bridge = CvBridge()
        self.cv_image = None
        
        self.subscription_cam = self.create_subscription(Image, '/image_raw', self.img_callback, 10)

        while self.cv_image is None and rclpy.ok():
            self.get_logger().info("Waiting for Camera image...")
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)  # CPU 사용률 최적화

        self.get_logger().info("Camera Ready --------------")

        self.cam_path = folder_path
        self.image_idx = 0

        # lidar 사전준비
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)

        self.ranges = None

        self.get_logger().info("Lidar Visualizer Ready ----------")

        lidar_path = os.path.join(folder_path, "lidar.txt")
        self.lidar_path = open(lidar_path, "w")

        # ultra 사전준비
        self.subscription = self.create_subscription(Int32MultiArray, 'xycar_ultrasonic', self.ultra_callback, 10)

        self.ultra_msg = None

        self.get_logger().info("Waiting for ultrasonic data...")
        self.wait_for_message()

        self.get_logger().info("Ultrasonic Ready ----------")
        
        ultra_path = os.path.join(folder_path, "ultra.txt")
        self.ultra_path = open(ultra_path, "w")
        
        # 루프
        ## camera 루프
        self.timer_cam = self.create_timer(0.5, self.process_images)

        ## lidar 루프
        self.timer_lidar = self.create_timer(0.5, self.lidar_timer_callback)

        ## ultra 루프
        self.timer_ultra = self.create_timer(0.5, self.ultra_timer_callback)

    def img_callback(self, data): self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    def lidar_callback(self, msg): self.ranges = np.array(msg.ranges[1:505])
    def ultra_callback(self, msg): self.ultra_msg = msg.data

    def process_images(self):
        cv2.imshow("original", self.cv_image)
        cv2.waitKey(1)

        img_path = os.path.join(
            self.cam_path, "image", f"{self.image_idx}.png"
        )
        cv2.imwrite(img_path, self.cv_image)
        self.image_idx += 1
    
    def lidar_timer_callback(self):
        """LiDAR 데이터를 주기적으로 업데이트하는 함수"""
        if self.ranges is not None:
            range_str=" ".join(str(rs) for rs in self.ranges)
            self.lidar_path.write(f"{range_str}\n")
            self.lidar_path.flush()

    def wait_for_message(self):
        while self.ultra_msg is None and rclpy.ok():
            rclpy.spin_once(self)  # 노드의 상태를 업데이트
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))  # 짧은 대기
    
    def ultra_timer_callback(self):
        if self.ultra_msg is not None:
            # self.get_logger().info(f"Ultrasonic Data: {self.ultra_msg}")
            self.ultra_path.write(f"{self.ultra_msg}\n")
            self.ultra_path.flush()

def create_save_folder(p_date="20250717"):
    try: base_dir = os.path.dirname(os.path.abspath(__file__))
    except NameError: base_dir = os.getcwd()

    # 현재 날짜 폴더명
    base_folder_name = p_date

    # 중복되지 않는 폴더명 찾기
    folder_name = base_folder_name
    counter = 0
    while os.path.exists(os.path.join(base_dir, folder_name)):
        counter += 1
        folder_name = f"{base_folder_name}({counter})"

    # 최종 경로
    folder_path = os.path.join(base_dir, folder_name)

    # 폴더 및 파일 생성
    os.makedirs(os.path.join(folder_path, "image"))
    return folder_path

def main(args=None):
    folder_path=create_save_folder()
    rclpy.init(args=args)
    main_node = MainNode(folder_path)

    try:
        rclpy.spin(main_node)
    except KeyboardInterrupt:
        pass
    finally:
        main_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
