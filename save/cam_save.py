#!/usr/bin/env python3

import cv2, time, os
import rclpy
from rclpy.node import Node
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CamTuneNode(Node):
    def __init__(self, save_path):
        super().__init__('cam_tune')
        self.bridge = CvBridge()
        self.cv_image = None

        # /image_raw 구독
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.img_callback,
            10
        )
        # 저장용 인덱스
        self.save_path = save_path
        self.image_idx = 0

        while self.cv_image is None and rclpy.ok():
            self.get_logger().info("Waiting for Camera image...")
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.get_logger().info("Camera Ready --------------")

        self.create_timer(0.5, self.process_images)


    def img_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def process_images(self):
        cv2.imshow("original", self.cv_image)
        cv2.waitKey(1)

        img_path = os.path.join(
            self.save_path, "image", f"{self.image_idx}.png"
        )
        cv2.imwrite(img_path, self.cv_image)
        self.get_logger().info(f"Saved image: {img_path}")
        self.image_idx += 1

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
    cam_path=create_save_folder()
    rclpy.init(args=args)
    node = CamTuneNode(cam_path)
    node.get_logger().info("Camera Viewer & Saver Ready --------------")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
