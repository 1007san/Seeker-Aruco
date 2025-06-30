#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

# ArUco dictionary type
aruco_type = "DICT_5X5_100"
ARUCO_DICT = {
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
}

# Get predefined dictionary with new API
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT["DICT_5X5_100"])

# Create detector parameters and detector object (new API)
arucoParams = cv2.aruco.DetectorParameters()
arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# Marker real size in meters (adjust as needed)
marker_size = 0.085

# Camera intrinsic matrix and distortion coefficients
# Replace with your calibrated values for accurate pose estimation
camera_matrix = np.array([[1642.1044917662514, 0, 556.3988211466784],
                          [0, 1642.1369306781667, 611.9199590653807],
                          [0, 0, 3.1813260637179797]], dtype=np.float64)
dist_coeffs = np.array([-0.10630235575660132, 0.7698914339751496, 
                        0.0016101930371934272, 0.0011709561966120813, 0.0],dtype=np.float64) #np.zeros((5, 1), dtype=np.float64)

# Initialize CvBridge for ROS <-> OpenCV image conversions
bridge = CvBridge()

def image_callback(msg):
    try:
        # Convert ROS Image to OpenCV BGR image
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Resize image for consistent processing
        h, w = img.shape[:2]
        width = 1088
        height = 1280
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)

        # Detect ArUco markers using the detector object
        corners, ids, rejected = arucoDetector.detectMarkers(img)

        if ids is not None:
            # Draw detected markers on the image
            img = cv2.aruco.drawDetectedMarkers(img, corners, ids)

            # Estimate pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_size, camera_matrix, dist_coeffs)

            # Draw axis and calculate distance for each marker
            for corner, rvec, tvec in zip(corners, rvecs, tvecs):
                # Draw the axes
                cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                # Calculate Euclidean distance
                distance = 0.23198 * np.linalg.norm(tvec)

                # Get the top-left corner of the marker to display distance
                corner = corner[0][0]  # Use the first corner point
                x, y = int(corner[0]), int(corner[1])

                # Display distance on the image
                cv2.putText(img, f"{distance:.2f} m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Show the resulting image
        cv2.imshow("ArUco Detection", img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            rospy.signal_shutdown("User requested shutdown.")

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")


if __name__ == "__main__":
    rospy.init_node("aruco_detector", anonymous=True)
    rospy.Subscriber("/fisheye/bleft/image_raw", Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()

