#!/usr/bin/env python3

import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver')
        self.motor_publisher = self.create_publisher(Float32MultiArray, 'xycar_motor', 1)
        self.motor_msg = Float32MultiArray()

        # 파라미터 초기화
        self.speed = 10
        self.angle = 0
        self.count = 0
        
        self.get_logger().info('----- Xycar self-driving node started -----')

        self.create_timer(0.1, self.main_loop)

    def drive(self, angle, speed):
        self.motor_msg.data = [float(angle), float(speed)]
        self.motor_publisher.publish(self.motor_msg)

    def main_loop(self):
        self.count+=1
        if self.count%25==0:
            if self.speed==10:
                self.speed=5
                self.angle=-self.angle
            else:
                self.speed=10
                self.angle=-self.angle
        self.drive(self.angle, self.speed)
            

def main(args=None):
    rclpy.init(args=args)
    driver_node = DriverNode()

    try:
        rclpy.spin(driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
