#!/usr/bin/env python3

import cv2, time
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CamTuneNode(Node):
    def __init__(self):
        # 'cam_tune'이라는 이름으로 노드 초기화
        super().__init__('cam_tune')
        self.bridge = CvBridge()
        self.cv_image = None
        
        # '/image_raw' 토픽 구독자 생성
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.img_callback,
            10  # 큐 크기 설정
        )
             
        # 카메라 토픽 도착 대기 (여기서 블로킹)
        while self.cv_image is None and rclpy.ok():
            self.get_logger().info("Waiting for Camera image...")
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)  # CPU 사용률 최적화
                
        self.get_logger().info("Camera Ready --------------")

    def img_callback(self, data):
        # 수신한 메시지를 OpenCV 이미지로 변환하여 저장
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    
    def run(self):
        # 이미지를 처리하는 루프
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)  # 메시지를 수신할 때까지 대기

            if self.cv_image is not None:
                # 수신된 이미지가 있을 때만 처리
                self.process_images()

    def process_images(self):
        # 이미지 출력
        cv2.imshow("original", self.cv_image)
        # GUI 처리를 위한 대기시간 (1ms)
        cv2.waitKey(1)

def main(args=None):
    # rclpy 초기화
    rclpy.init(args=args)

    # CamTuneNode 인스턴스 생성
    node = CamTuneNode()
    node.get_logger().info("Camera Viewer Ready --------------")

    # run 메서드 실행 (영상 처리 루프)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 종료 및 OpenCV 창 닫기
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
