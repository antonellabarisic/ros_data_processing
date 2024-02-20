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
        self.image_sub = rospy.Subscriber("/hawkred/camera/color/image_raw", Image, self.callback)
        self.destination_path = "/home/antonella/Datasets/mbzirc2024/small_large_object/images_from_bags"
        self.save_rate = 0.5  # Save an image every 0.1 seconds
        self.last_save_time = 0
        self.rotate_180 = False

        if not os.path.exists(self.destination_path):
            os.makedirs(self.destination_path)

        self.img_count = 0

    def callback(self, data):
        current_time = time.time()  # Get current time in seconds
        if current_time - self.last_save_time >= self.save_rate:
            try:
                if self.rotate_180:
                        cv_image = cv2.rotate(self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8"), cv2.ROTATE_180)
                else:
                    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            # Name image based on rospy time
            current_ros_time = rospy.get_rostime()  # This returns a rospy.Time instance
            ros_time_sec = current_ros_time.secs
            ros_time_nsec = current_ros_time.nsecs

            img_name = "image_{}_{}.png".format(ros_time_sec, ros_time_nsec)

            cv2.imwrite(os.path.join(self.destination_path, img_name), cv_image)
            self.img_count += 1
            self.last_save_time = current_time  # Update the last save time

            print("Saved image named {}".format(img_name))
            print("Saved image number {}".format(self.img_count))


def main():
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

