#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class BlueObjectPublisher(Node):
    def __init__(self):
        super().__init__('blue_object_publisher')
        self.publisher_ = self.create_publisher(Image, 'blue_object_coordinates', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.03, self.publish_coordinates)

    def publish_coordinates(self):
        ret, frame = self.cap.read()
        frame = cv2.resize(frame, (640, 480))

        if not ret:
            self.get_logger().error("Error reading frame from webcam")
            return

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])

        mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            max_contour = max(contours, key=cv2.contourArea)

            if max_contour is not None:
                contour_area = cv2.contourArea(max_contour)
                if contour_area > 400:
                    M = cv2.moments(max_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        json_data={"x":(int)((320-cx)/20),"y":(int)((240-cy)/20)}  # Convert to desired coordinates
                        json_send=json.dumps(json_data)

                        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        self.publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BlueObjectPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
