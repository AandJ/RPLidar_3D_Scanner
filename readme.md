# 3D scanner made using A1M1 RPLIDAR

## Introduction

This code was written by Jonathan Wheadon and Ijaas at the University of Plymouth (UK) as part of the ROCO318 module.  

This code was done as part of a **SHORT** project and therfore was not fully thought through, there are many better ways to achieve this result.
If anyone wishes to take this code and improve it feel free and message me so if you want me to link your improved version in this REPO.  

This code takes use of the RPLIDAR SDK provided by the makers of the A1M1 (slamtec) and is built upon the Code by Nicholas Lui (https://github.com/NickL77/RPLidar_Hector_SLAM)  

Dependencies : PCL (point cloud library), ROS (Robot operating system)  

<p align="center">
<a href="https://www.youtube.com/watch?v=Y31oJjL61w0" target="_blank"><img src="http://img.youtube.com/vi/Y31oJjL61w0/0.jpg" alt="Video of Final Product" width="640" height="360" border="0" /></a>  
</p>  

## Brief rundown of the code

The code we wrote can be found Here [ScanAssemble.cpp](https://github.com/AandJ/RPLidar_3D_Scanner/blob/master/RPlidar_3D_Scanner/src/ScanAssemble.cpp)  

The code we wrote (ScanAssemble.cpp) starts by subscribing to the "/scan" topic which is part of the launch file "rplidar.launch" provided by the SDK for the RPLIDAR, we then convert this to a point cloud and use an Affine transformation matrix to rotate the cloud to be the same as the current rotation of the RPLIDAR, this is then concatenated with all previous scans, this is then published to RVIZ and the control data is published to "/control" which the Arduino subscribes too, this then rotates the RPLIDAR to its next angle to take another scan. Once this has been done 180 times a full scan has been complete and the rotation changes direction and restarts  

## How to use the code...

You will need an arduino (w/Motor sheild and ROS), stepper motor, 3d printer (to make case), rpldar A1 series and a laptop running ubuntu (other linux based OS should work but have not been tested).  
First set the up arduino with the motor shield and the stepper motor, download stepperCONTROL.ino to the arduino via the arduino IDE, make sure the motor is in the case, some modifications to the case may be needed depending on which stepper motor you have opted to use.  

UNZIP and place the folder in the SRC of a catkin workspace.  

Via the terminal navigate to the top level folder of the catkin workspace and use the following comands.  

``` cpp
source devel/setup.bash
catkin_make
```  


You will need to have uploaded the stepperCONTROL.ino Program to the arduino via the arduino IDE and ran rosserial as provided by ROS.  

``` cpp
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
```  

Note that ttyACM0 may change depending on which usb port you are using.  

The launch file ALL.launch should launch both the RPLIDAR SKD ros node aswell as the node to handle the incoming scans and output them to RVIZ.  

``` cpp
source devel/setup.bash
roslaunch rplidar_ros ALL.launch
```  



