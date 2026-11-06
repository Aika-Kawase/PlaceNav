#!/usr/bin/env python3
"""Decode recorded C2 JPEG/PNG without changing its acquisition header."""
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image

class Decoder:
    def __init__(self):
        self.pub = rospy.Publisher(rospy.get_param('~output_topic', '/fusion/c2/image_raw'), Image, queue_size=1)
        self.sub = rospy.Subscriber(rospy.get_param('~input_topic', '/usb_cam/image_raw/compressed'),
                                    CompressedImage, self.receive, queue_size=1, buff_size=16 * 1024 * 1024)

    def receive(self, msg):
        try:
            image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('image decode returned no data')
            out = Image()
            out.header = msg.header
            out.height, out.width = image.shape[:2]
            out.encoding = 'bgr8'
            out.is_bigendian = False
            out.step = out.width * 3
            out.data = image.tobytes()
            self.pub.publish(out)
        except (ValueError, cv2.error) as error:
            rospy.logwarn_throttle(2, 'C2 compressed image rejected: %s', error)

if __name__ == '__main__':
    rospy.init_node('decompress_c2')
    decoder = Decoder()
    rospy.spin()
