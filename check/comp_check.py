#!/usr/bin/env python3

import cv2, time
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

        ## lidar gui
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlim(-150, 150)
        self.ax.set_ylim(-150, 150)
        self.ax.set_aspect('equal')
        self.lidar_points, = self.ax.plot([], [], 'bo')

        plt.ion()  # 인터랙티브 모드 활성화
        plt.show()

        self.get_logger().info("Lidar Visualizer Ready ----------")

        # ultra 사전준비
        self.subscription = self.create_subscription(Int32MultiArray, 'xycar_ultrasonic', self.ultra_callback, 10)

        self.ultra_msg = None

        self.get_logger().info("Waiting for ultrasonic data...")
        self.wait_for_message()

        self.get_logger().info("Ultrasonic Ready ----------")
        
        # 루프
        ## camera 루프
        self.timer_cam = self.create_timer(1, self.process_images)

        ## lidar 루프
        self.timer_lidar = self.create_timer(1, self.lidar_timer_callback)

        ## ultra 루프
        self.timer_ultra = self.create_timer(1.0, self.ultra_timer_callback)

    def img_callback(self, data): self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    def lidar_callback(self, msg): self.ranges = np.array(msg.ranges[1:505])
    def ultra_callback(self, msg): self.ultra_msg = msg.data

    def process_images(self):
        cv2.imshow("original", self.cv_image)
        cv2.waitKey(1)
    
    def lidar_timer_callback(self):
        """LiDAR 데이터를 주기적으로 업데이트하는 함수"""
        if self.ranges is not None:
            # 각도 계산 (0 ~ 2π로 변환, -90도 보정)
            angles = np.linspace(0, 2 * np.pi, len(self.ranges)) - np.pi / 2

            # 극좌표 -> 직교좌표 변환 (cm 단위)
            x = self.ranges * np.cos(angles) * 100
            y = self.ranges * np.sin(angles) * 100

            # 그래프 업데이트
            self.lidar_points.set_data(x, y)
            self.fig.canvas.draw_idle()
            plt.pause(0.01)

            # 샘플 데이터 출력 (36개 샘플)
            ranges_cm = [int(distance * 100) for distance in self.ranges]
            step = len(ranges_cm) // 36
            self.get_logger().info(f"Selected distance values (cm): {ranges_cm[::step]}")

    def wait_for_message(self):
        while self.ultra_msg is None and rclpy.ok():
            rclpy.spin_once(self)  # 노드의 상태를 업데이트
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))  # 짧은 대기
    
    def ultra_timer_callback(self):
        if self.ultra_msg is not None:
            self.get_logger().info(f"Ultrasonic Data: {self.ultra_msg}")


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
