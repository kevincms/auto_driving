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
from std_msgs.msg import Float32MultiArray
import labacon_drive

class MainNode(Node):
    def __init__(self):
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

        # lidar 사전준비
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)

        self.ranges = None

        self.get_logger().info("Lidar Visualizer Ready ----------")

        # ultra 사전준비
        self.subscription = self.create_subscription(Int32MultiArray, 'xycar_ultrasonic', self.ultra_callback, 10)

        self.ultra_msg = None

        self.get_logger().info("Waiting for ultrasonic data...")
        self.wait_for_message()

        self.get_logger().info("Ultrasonic Ready ----------")

        # motor 사전준비
        self.motor_publisher = self.create_publisher(Float32MultiArray, 'xycar_motor', 1)
        self.motor_msg = Float32MultiArray()

        self.speed = 7
        self.angle = 0
        
        self.get_logger().info('----- Xycar self-driving node started -----')
        
        # camera 변수 : self.cv_image
        # lidar 루프 : self.ranges
        # ultra 루프 : self.ultra_msg

        self.labacon_node=labacon_drive.Labacon_node()
        main_save_var = self.create_timer(0.1, self.main_loop)

    def main_loop(self):
        self.angle=self.labacon_node.labacon_main(self)
        self.drive(self.angle,self.speed)

    def drive(self, angle, speed):
        self.motor_msg.data = [float(angle), float(speed)]
        self.motor_publisher.publish(self.motor_msg)

    def img_callback(self, data): self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    def lidar_callback(self, msg): self.ranges = np.array(msg.ranges[1:505])
    def ultra_callback(self, msg): self.ultra_msg = msg.data

    def wait_for_message(self):
        while self.ultra_msg is None and rclpy.ok():
            rclpy.spin_once(self)  # 노드의 상태를 업데이트
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))  # 짧은 대기

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
    rclpy.init(args=args)
    main_node = MainNode()
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
