#!/usr/bin/env python3

import cv2
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

def rosbag_to_video(rosbag_path, video_path, topic_name, fps=30):
    print("Starting ROS bag to video conversion")
    bridge = CvBridge()
    bag = rosbag.Bag(rosbag_path, "r")
    msg_count = bag.get_message_count(topic_filters=[topic_name])
    duration = bag.get_end_time() - bag.get_start_time()

    calculated_fps = msg_count / duration
    print(f"Message count: {msg_count}")
    print(f"Duration: {duration}")
    print(f"Calculated FPS: {calculated_fps}")

    first_frame_initialized = False
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        try:

            if msg._type == 'sensor_msgs/Image':
                try:
                    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
                except Exception as e:
                    print("Failed to convert sensor_msgs/Image:", e)
            elif msg._type == 'sensor_msgs/CompressedImage':
                try:
                    frame = bridge.compressed_imgmsg_to_cv2(msg)
                except Exception as e:
                    print("Failed to convert sensor_msgs/CompressedImage:", e)
    
            if not first_frame_initialized:
                height, width = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # For MP4 file
                out = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
                first_frame_initialized = True

            out.write(frame)
        except Exception as e:
            print(e)
            continue

    if not first_frame_initialized:
        print("Could not initialize video writer - ensure the rosbag contains valid image messages.")
    else:
        out.release()
        print("Conversion completed successfully!")

    bag.close()

if __name__ == '__main__':
    rosbag_path = '/home/antonella/bags/red_only_epoch110.bag'
    video_path = '/home/antonella/bags/red_only_epoch110.mp4'
    topic_name = '/detected_objects/compressed' 
    # topic_name = '/yolov5/image_out/compressed'
    fps = 7

    rosbag_to_video(rosbag_path, video_path, topic_name, fps)
