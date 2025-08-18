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
from xycar_msgs.msg import XycarMotor
import test

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
        self.motor_publisher = self.create_publisher(XycarMotor, 'xycar_motor', 1)
        self.motor_msg = XycarMotor()

        self.speed = 10
        self.angle = 0
        
        self.get_logger().info('----- Xycar self-driving node started -----')
        
        # camera 변수 : self.cv_image
        # lidar 루프 : self.ranges
        # ultra 루프 : self.ultra_msg

        main_save_var = self.create_timer(0.1, self.main_loop)

    def main_loop(self):
        test.test_main_once(self)

    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor_publisher.publish(self.motor_msg)

    def img_callback(self, data): self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    def lidar_callback(self, msg): self.ranges = np.array(msg.ranges[1:505])
    def ultra_callback(self, msg): self.ultra_msg = msg.data

    def wait_for_message(self):
        while self.ultra_msg is None and rclpy.ok():
            rclpy.spin_once(self)  # 노드의 상태를 업데이트
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))  # 짧은 대기

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
