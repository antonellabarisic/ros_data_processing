#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

def video_to_ros(video_path, publish_rate, new_topic_name):
    print("Starting video to ROS conversion")
    # Create the publisher
    pub = rospy.Publisher(new_topic_name, Image, queue_size=10)
    rate = rospy.Rate(publish_rate)  # 10hz

    # Create a CvBridge instance
    bridge = CvBridge()

    # Open the video file
    cap = cv2.VideoCapture(video_path)
    print("Video file opened")

    while not rospy.is_shutdown() and cap.isOpened():
        print("Publishing video frames")
        ret, frame = cap.read()
        if ret:
            # Convert the image frame to a ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")

            # Publish the image
            pub.publish(ros_image)

            # Sleep to maintain a consistent rate
            rate.sleep()
        else:
            break

    # Release the video capture object
    cap.release()

def video_to_ros_compressed(video_path, publish_rate, new_topic_name):
    print("Starting video to ROS compressed conversion")

    # Create an image transport publisher instead of a regular publisher
    pub = rospy.Publisher(new_topic_name + "/compressed", CompressedImage, queue_size=10)

    # Create a CvBridge instance
    bridge = CvBridge()

    # Open the video file
    cap = cv2.VideoCapture(video_path)
    print("Video file opened")

    rate = rospy.Rate(publish_rate)  # Publish rate

    while not rospy.is_shutdown() and cap.isOpened():
        print("Publishing compressed video frames")
        ret, frame = cap.read()
        if ret:
            # Convert the image frame to a ROS CompressedImage message
            ros_compressed_image = bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')

            # Publish the compressed image
            pub.publish(ros_compressed_image)

            # Sleep to maintain a consistent rate
            rate.sleep()
        else:
            break

    # Release the video capture object
    cap.release()

if __name__ == '__main__':
    rospy.init_node("video_publisher", anonymous=True)
    print("ROS node initialized")

    # video_path = '/home/antonella/ros/catkin_ws/src/ros_data_processing/data/DJI_0018.MP4'
    # rate = 30 # Hz

    video_path = rospy.get_param('~video_path', '/home/antonella/ros/catkin_ws/src/ros_data_processing/data/DJI_0018.MP4')
    rate = rospy.get_param('~rate', 10)
    new_topic_name = rospy.get_param('~new_topic_name', '/camera/image_raw')
    compress = rospy.get_param('~compress', False)
    print("Video path: {}".format(video_path))
    print("Publish rate: {}".format(rate))
    print("New topic name: {}".format(new_topic_name))
    print("Compress: {}".format(compress))

    try:
        if compress:
            video_to_ros_compressed(video_path, rate, new_topic_name)
        else:
            video_to_ros(video_path, rate, new_topic_name)
    except rospy.ROSInterruptException:
        pass
