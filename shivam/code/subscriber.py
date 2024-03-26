#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json

class BlueObjectSubscriber(Node):
    def __init__(self):
        super().__init__('blue_object_subscriber')
        self.subscription = self.create_subscription(Image, 'blue_object_coordinates', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Extract JSON data from Image message
        json_data = json.loads(msg.data)

        # Process the received coordinates here
        # For demonstration, just print the received JSON data
        print(json_data)

def main(args=None):
    rclpy.init(args=args)
    node = BlueObjectSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
