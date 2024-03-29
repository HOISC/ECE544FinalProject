#! /usr/bin/env/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import serial


class SubscribeLidarNode(Node):
    def __init__(self):
        super().__init__("subscribe_lidar_node")
        self.publisher_ = self.create_publisher(LaserScan, 'lidar_data', 10)
        self.subsciber = self.create_subscription(
            LaserScan, "scan", self.callback_process_lidar, 100
        )
        self.get_logger().info("Subscriber has started")

        #self.serial = serial.Serial('/dev/ttyACM0', baudrate=9600)
    
    def callback_process_lidar(self, msg):
        msg_ = LaserScan()
        
        PI = 3.14159
        min_angle = (msg.angle_min*180)/PI
        angle_inc = (msg.angle_increment*180)/PI
        end_angle = (msg.angle_max*180)/PI
        #self.get_logger().info("NEW DATA ********************* NEW DATA *********************** NEW DATA ********************** NEW DATA ********************")
        count_to_write = b''
        count = 0
        indxMsg_ = 0
        for count in range(0, len(msg.ranges)):
            angle = min_angle + (count * angle_inc)
            if angle > 360:
                angle = angle - 360
            if (angle >= 0 and angle <= 180):
                msg_.ranges.append(msg.ranges[count])
                msg_.intensities.append(angle)

                #indxMsg_ = indxMsg_ + 1
                #self.get_logger().info(str(msg.ranges[count]) + " m" + "             " + str(angle) + " deg  ")
        self.get_logger().info("Lengths: " + str(len(msg_.ranges)) + "  " + str(len(msg_.intensities)))

        #self.get_logger().info("\n\r\n\rDATA: " + str(msg_.data))
        #self.get_logger().info("\n\r\n\rAngles: " + str(msg_.angle))
        self.publisher_.publish(msg_)
        #count_to_write = ('F15!L42@').encode('ascii')
        #self.serial.write(count_to_write)
        # count_to_write = ('F10!').encode('ascii')
        # self.serial.write(count_to_write)
    

def main(args=None):
    rclpy.init(args=args)
    node = SubscribeLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()