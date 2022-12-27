# aruco_detection
## aruco detector 
This package detects any aruco markers(registered in the dictionary) present in the frame of the OCam.

## pose estimator
This package calculates the pose of the detected aruco marker with respect to the world frame.

# Steps for installation

### Installing dependencies
```bash
sudo apt-get install -y libopencv-dev python3-opencv ros-noetic-cv-bridge ros-noetic-codec-image-transport ros-noetic-gazebo-msgs ros-noetic-usb-cam libudev-dev libv4l-dev```

#### If issues in libopencv-dev
```bash 
sudo apt purge libgphoto2-6 && sudo apt autoremove
sudo apt install libgphoto2-6
sudo apt install  libopencv-calib3d-dev libopencv-contrib-dev libopencv-features2d-dev libopencv-highgui-dev libopencv-imgcodecs-dev  libopencv-objdetect-dev  libopencv-shape-dev libopencv-stitching-dev  libopencv-superres-dev  libopencv-video-dev libopencv-videoio-dev  libopencv-videostab-dev  libopencv4.2-java libopencv-calib3d4.2 libopencv-contrib4.2 libopencv-features2d4.2 libopencv-highgui4.2 libopencv-imgcodecs4.2 libopencv-videoio4.2 libgdal26 libodbc1 libgphoto2-dev```

### Cloning and building repo
```bash
mkdir -p ~/shastra_ws/src
cd shastra_ws 
catkin init
cd src
git clone https://github.com/AerialRobotics-IITK/shastra23 .
catkin build```
