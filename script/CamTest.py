#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("Video Stream", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(f"Failed to process image: {e}")

if __name__ == "__main__":
    rospy.init_node("video_stream_extractor")
    rospy.Subscriber("/left/left/image_raw", Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()

