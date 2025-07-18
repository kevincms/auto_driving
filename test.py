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

def test_main_once(obj):
    ## camera 변수 : self.cv_image
    ## lidar 루프 : self.ranges
    ## ultra 루프 : self.ultra_msg
    ## self.speed, speed.obj 접근하면 안됨
    
    # 사용하기 전 반드시 None 인지 확인 후 예외처리로 코드 작성하기
    if obj.cv_image is not None:
        test_cam(obj)    

    if obj.ranges is not None:
        print(obj.ranges[0])
    
    if obj.ultra_msg is not None:
        print(obj.ultra_msg)

def test_cam(obj):
    cv2.imshow("original", obj.cv_image)
    cv2.waitKey(1)