# Seeker-Aruco
Detecting Aruco Tag with Seeker Omni-D Camera

Seeker repository: https://gitee.com/nochain/seeker1


# 1.  Environmental requirements

 - Ubuntu 20.04
 - ROS Noetic (Installation Tutorial)
 - sudo apt install libusb-1.0-0-dev
 - pip3 install pyusb numpy ruamel.yaml



# 2.  Installation steps

1. Create workspace：

       mkdir -p ~/catkin_ws/src


2. Clone repository:

       cd ~/catkin_ws/src && git clone http://github.com/1007san/Seeker-Aruco.git


3. Compile Project：

       catkin_make -DCMAKE_BUILD_TYPE=Release


4. Set environment variables

       source ~/catkin_ws/devel/setup.bash

       OR：echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


5. Install undistortion package
  
   For AMD:

       dpkg -i ~/catkin_ws/src/seeker1/deb/ros-noetic-image-undistort_0.0.0-0focal_amd64.deb
    
   For ARM:
    
       dpkg -i ~/catkin_ws/src/seeker1/deb/ros-noetic-image-undistort_0.0.0-0focal_arm64.deb



# 3.  Hardware connection

1. Connect the device using a USB 3.0 Type-C cable
2. Set device permissions to sudo vim /etc/udev/rules.d/99-seeker.rules, add udev rules

       SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0000", MODE="0666"


3.  ‌Reload udev rules‌

           sudo udevadm control --reload && sudo udevadm trigger


4.  Then plug and unplug the device


    
# 4.  Quick start

Basic data stream (fisheye image+IMU+disparity map)

        roslaunch seeker 1seeker_nodelet.launch


**Preparation before running other routines: (reading calibration parameters)**

Ensure that the module can be used normally and there are no issues with USB access permissions. Execute the following command to read calibration parameters from the module and generate a configuration file:

        python3 ~/catkin_ws/src/seeker/scripts/1get_kalibr_info.py
        
-  Output file path：The generated calibration parameter file is saved by default to /tmp/kalibr_cam_chain.yaml


3.3.3. Configuration file deployment

-    Copy the generated calibration file to the target configuration directory:

            cp /tmp/kalibr_cam_chain.yaml ~/catkin_ws/src/seeker/config/seeker_omni_depth/

-    Dependency Relationship：Subsequent splicing, distortion resolution, and other processes need to be initialized with parameters based on this file.



# 5.  Launch file description

**Image viewing**

1.  fisheye view

![Screenshot from 2025-06-30 18-25-54](https://github.com/user-attachments/assets/d259ce6d-1ad8-4582-b0e4-7a2849a27c81)

        #Start the basic data stream (fisheye image+disparity map+IMU)
    
        roslaunch seeker 1seeker_nodelet.launch  

rqt_gui import

rosrun rqt_gui rqt_gui, then click import in perspectives to import all gui files of ~/catkinw_s/src/seeker/gui one by one. Then select the seeker perspective. View the original image and disparity map.


2.  undistorted view

![Screenshot from 2025-06-30 18-26-08](https://github.com/user-attachments/assets/f0d74a41-84c1-4204-b881-f875fd3e87cc)
  
        roslaunch seeker 3undistort_nodelet.launch

        rosrun rqt_gui rqt_gui

Select the undistort.perspective after importing it.


3.  All view

![Screenshot from 2025-06-30 18-26-27](https://github.com/user-attachments/assets/d593a257-96b8-4cfc-8788-e7ec3ccf38bf)

        roslaunch seeker allview.launch

        rosrun rqt_gui rqt_gui

Select the AllView.perspective


        
# 6.  Detecting Aruco Tags

All python scripts are written to detect the *"DICT_5X5_100": cv2.aruco.DICT_5X5_100* dictionary. This can be changed in the python scripts.

Launch the all view to publish all video streams.

        roslaunch seeker allview.launch


1.    Testing undistortion of fisheye views

![Video Stream_screenshot_30 06 20251](https://github.com/user-attachments/assets/327384ce-b818-4161-83ad-12e768fe3973)

        rosrun seeker CamTest.py

Subscribes to /left/left/image_raw and displays the undistorted view. This topic can be changed in the python script to view the desired lens.


2.    Testing detection of Aruco tag and orientation

![ArUco Detection_screenshot_30 06 20252](https://github.com/user-attachments/assets/a0e69b18-df16-4800-bb3b-8b3a8d1c6167)

        rosrun seeker arucotest.py

Subscribes to /fisheye/bleft/image_raw. This topic can be changed in the python script to view the desired lens.


3.    Testing the distance and orientation of the Aruco Tag

![ArUco Detection_screenshot_30 06 20253](https://github.com/user-attachments/assets/312c4593-1684-440a-9eb5-62832f306061)

        rosrun seeker distancetest.py

Subscribes to /fisheye/bleft/image_raw. This topic can be changed in the python script to view the desired lens.

.

.

.

*To be added:*

A dashboard to view all 4 fisheye views, detect up to a total of 10 Aruco Tags in all 4 directions, with each ID Aruco Tag's position printed out.
