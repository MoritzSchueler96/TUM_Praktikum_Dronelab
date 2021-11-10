## Install Calibration Toolchain
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
rosdep install camera_calibration
sudo nano /opt/ros/noetic/lib/camera_calibration/cameracalibrator.py
    Change first line from: "!/usr/bin/python" to "!/usr/bin/python3"

## Run calibration
start terminal
roscore
start terminal 
rosbag play calibration_sequence_chessboard.bag

start calibration
rosrun camera_calibration cameracalibrator.py --size 5x5 --square 0.16 \
--no-service-check image:=/ardrone/front/image_raw \
camera:=/ardrone/front

## Calibration Results
camera matrix
582.047949 0.000000 316.339374
0.000000 579.300111 205.835304
0.000000 0.000000 1.000000

distortion
-0.562099 0.358005 -0.011755 -0.001345 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
476.741974 0.000000 313.134004 0.000000
0.000000 545.104370 205.975888 0.000000
0.000000 0.000000 1.000000 0.000000