# Project-UAV-software-senor fusion of camera and Lidar 

if you want to run with rviz2 
step1
------------------------------------------------------------------
connect to usb port
make sure both usb are connected  and power is on TTL has to be on 2500000 side
---------------------------------------------------------------------

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar_a2m12_launch.py
-------------------------------------------------------------------------
----------------------------------------------------------------------------

if lidar a2m12doesnt work or accidentley taken out of usb port 
Unplug + replug the LiDAR, then run:
-------------------------------------------------------------------
dmesg | tail -n 60
ls -l /dev/ttyUSB* /dev/ttyACM*
------------------------------------------------------------------------------
------------------------------------------------------------------------------
to coonect intel D435i camera 
step 1
---------------------------------------------------------------------------------------------
source /opt/ros/humble/setup.bash
	ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
------------------------------------------------------------------------------------------------
  step 2
  ------------------------------------------------------------------------------------
  source /opt/ros/humble/setup.bash
  ------------------------------------------------------------------------------------------
  step 3
  -----------------------------------------------------------------------------------------
  rviz2
  --------------------------------------------------------------------

  
