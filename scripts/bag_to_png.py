#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/red/camera/image_color", Image, self.callback)
        self.image_sub = rospy.Subscriber("/throttled_camera_topic", Image, self.callback)
        self.destination_path = "/home/antonella/Datasets/mbzirc2024/abu_dhabi_yellow_boat/1501_from_bags"
        self.save_rate = 0.1  # Save an image every 5 seconds
        self.last_save_time = 0
        self.rotate_180 = True

        if not os.path.exists(self.destination_path):
            os.makedirs(self.destination_path)

    def callback(self, data):
        # current_time = time.time()
        # if current_time - self.last_save_time >= self.save_rate:
        try:
            if self.rotate_180:
                    cv_image = cv2.rotate(self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8"), cv2.ROTATE_180)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        ros_time_int = int(rospy.get_time())
        img_name = "image_{}.png".format(ros_time_int)
        cv2.imwrite(os.path.join(self.destination_path, img_name), cv_image)
        # self.last_save_time = current_time

def main():
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

