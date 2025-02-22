#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from ur5e_vision_msgs.msg import Tracker

tracker_msg = Tracker()
class UR5Vision(Node):
    def __init__(self):
        super().__init__('ur5_vision')

        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()

        # Create subscriber for the camera image
        self.image_sub = self.create_subscription(
            Image,
            '/color/image_raw',
            self.image_callback,
            10  # QoS history depth
        )

        # Create publisher for the tracked object
        self.cxy_pub = self.create_publisher(Tracker, '/objects', 10)

        self.get_logger().info("UR5 Vision Node has been started")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert image to HSV format
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define green color range and create a mask
        lower_green = np.array([32, 50, 50])
        upper_green = np.array([75, 255, 255])
        # Define gray color range and create a mask
        lower_gray = np.array([0, 0, 100])
        upper_gray = np.array([179, 40, 200])
        mask = cv2.inRange(hsv, lower_green, upper_green) # 837,150
        # mask = cv2.inRange(hsv, lower_gray, upper_gray) # 722,584

        # Find contours
        cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Get image dimensions
        h, w, d = image.shape

        # Compute object centroid using moments
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            for i, c in enumerate(cnts):
                area = cv2.contourArea(c)
                # import pdb; pdb.set_trace()
                if area > 5000:
                    self.track_flag = True
                    self.cx = cx
                    self.cy = cy
                    self.error_x = self.cx - (w / 2 + 182)
                    self.error_y = self.cy - (h / 2 + 224)

                    tracker_msg.x = float(cx)
                    tracker_msg.y = float(cy)
                    tracker_msg.flag1 = self.track_flag
                    tracker_msg.error_x = self.error_x
                    tracker_msg.error_y = self.error_y

                    # Draw a circle at the detected object position
                    cv2.circle(image, (cx, cy), 10, (0, 0, 0), -1)
                    cv2.putText(
                        image, 
                        f"({cx}, {cy})", 
                        (cx - 5, cy + 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, 
                        (255, 255, 255), 
                        1
                    )
                    cv2.drawContours(image, cnts, -1, (255, 255, 255), 1)

                    break
                else:
                    self.track_flag = False
                    tracker_msg.flag1 = self.track_flag


        # Show the processed image
        self.cxy_pub.publish(tracker_msg)
        cv2.imshow("Tracking Window", image)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    vision_node = UR5Vision()
    rclpy.spin(vision_node)

    # Cleanup
    vision_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
