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

  

## UAV propeller design and performance tool

A preliminary propeller design/performance estimator is available at:

- `tools/propeller_performance_tool.py`

It computes:
- Blade chord and twist distributions along radius
- Section aerodynamic state (`phi`, `alpha`, `Re`, `CL`, `CD`)
- Section contributions (`dT`, `dQ`)
- Total thrust, torque, shaft power
- Coefficients `CT`, `CP`, `CQ`
- Figure of merit and induced velocity estimate

Example usage:

```bash
python3 tools/propeller_performance_tool.py \
  --diameter-m 0.254 \
  --pitch-at-75-m 0.1143 \
  --rpm 6500 \
  --flight-speed-mps 0 \
  --json-out prop_report.json
```

Tune these parameters with your motor/airfoil/bench conditions:
- `--cl-alpha-per-rad`, `--cl-max`, `--cd0`, `--k-induced-drag`
- `--root-chord-m`, `--tip-chord-m`, `--radial-sections`
- `--rho`, `--mu`

> Note: This is a first-pass engineering model for design iteration and test planning, not a certified aerodynamic solver.
