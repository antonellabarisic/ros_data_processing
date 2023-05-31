#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

"""
Flip an image stream.

Author: sam.pfeiffer at conigital.com
        Antonella Barisic at UNIZG-FER
"""


class ImageFlipper(object):
    def __init__(self, flip_mode, image_topic, queue_size=10):
        """
        flip_mode int: opencv flip mode, 0 == horizontal, 1 == vertical, -1 both
        """
        self._flip_mode = flip_mode
        self._cv_bridge = CvBridge()
        self._initialized = False
        self._sub = rospy.Subscriber(image_topic, Image, self._image_cb,
                                     queue_size=queue_size)
        output_topic = self._sub.resolved_name + '_flipped'
        self._pub = rospy.Publisher(output_topic, Image,
                                    queue_size=queue_size)
        # Python does not have image transport to do it for us
        self._compressed_pub = rospy.Publisher(output_topic + "/compressed", CompressedImage,
                                               queue_size=queue_size)
        rospy.loginfo("Flipping topic: {} into topic: {}".format(
            self._sub.resolved_name, self._pub.resolved_name))
        self._initialized = True

    def _image_cb(self, image):
        if not self._initialized:
            return

        if self._pub.get_num_connections() > 0 or self._compressed_pub.get_num_connections() > 0:
            cv2_img = self._cv_bridge.imgmsg_to_cv2(image)
            flipped = cv2.flip(cv2_img, self._flip_mode)
            # check encoding
            rospy.logwarn("Image encoding: {}".format(image.encoding))
            if image.encoding == 'rgb8':
                out_img_msg = self._cv_bridge.cv2_to_imgmsg(flipped,
                                            encoding='rgb8')
            elif image.encoding == 'bgr8':
                out_img_msg = self._cv_bridge.cv2_to_imgmsg(flipped,
                                            encoding='rgb8')
            elif image.encoding == 'mono16':
                out_img_msg = self._cv_bridge.cv2_to_imgmsg(flipped,
                                            encoding='mono16')
            elif image.encoding == '16UC1':
                out_img_msg = self._cv_bridge.cv2_to_imgmsg(flipped,
                                            encoding='mono16')
            else:
                rospy.logwarn("Image encoding not supported, got: {}".format(
                    image.encoding))
            out_img_msg.header = image.header
            if self._pub.get_num_connections() > 0:
                self._pub.publish(out_img_msg)
            if self._compressed_pub.get_num_connections() > 0:
                cimg_msg = self._cv_bridge.cv2_to_compressed_imgmsg(flipped,
                                                            dst_format='jpg')
                cimg_msg.header = image.header
                self._compressed_pub.publish(cimg_msg)


if __name__ == '__main__':
    rospy.init_node('image_flipper', anonymous=True)
    argv = rospy.myargv(sys.argv)

    if len(argv) > 2:
        flip_mode = argv[1]
        image_topic = argv[2]
    elif len(argv) > 1:
        flip_mode = argv[1]
        image_topic = "/camera/image_raw"
    else:
        raise RuntimeError(
            "{} requires argument to flip 'horizontal', 'vertical', or 'both'".format(argv[0]))
    # horizontal
    if flip_mode.startswith('h'):
        flip_mode_num = 0
    # vertical
    elif flip_mode.startswith('v'):
        flip_mode_num = 1
    # both
    elif flip_mode.startswith('b'):
        flip_mode_num = -1
    else:
        raise RuntimeError(
            "Expected argument to flip 'horizontal', 'vertical', or 'both', got: {}".format(flip_mode))
    ImageFlipper(flip_mode_num, image_topic)
    rospy.spin()
