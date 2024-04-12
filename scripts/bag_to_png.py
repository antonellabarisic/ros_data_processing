#!/usr/bin/env python3
import rosbag
import cv2
from cv_bridge import CvBridge
import os

def extract_images_from_rosbag(rosbag_file, output_folder, image_topic_name):
    # Create output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Initialize the CV bridge
    bridge = CvBridge()

    idx = 0
    # Open the ROS bag file
    with rosbag.Bag(rosbag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[image_topic_name]):
            try:
                if msg._type == 'sensor_msgs/Image':
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                elif msg._type == 'sensor_msgs/CompressedImage':
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg)
                
                # Construct the filename using the timestamp of the message
                filename = os.path.join(output_folder, '{:04}.png'.format(idx))
                idx += 1
                
                # Save the image
                cv2.imwrite(filename, cv_image)
                print("Saved image", filename)
            except Exception as e:
                print("Failed to convert and save image:", e)

if __name__ == '__main__':
    rosbag_file = '/home/antonella/bags/mbzirc_nella/first_test__2024-01-18-07-54-17/red_only_epoch110.bag'
    output_folder = '/home/antonella/bags/mbzirc_nella/first_test__2024-01-18-07-54-17/red_only_epoch110/'
    image_topic_name = '/detected_objects/compressed'
    extract_images_from_rosbag(rosbag_file, output_folder, image_topic_name)
