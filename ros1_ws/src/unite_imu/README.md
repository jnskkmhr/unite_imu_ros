# ROS node for unite-imu method
This is ROS node for 
* interpolating accel message to the gyro's timestamp
* fusing interpolated accel and gyro value into single imu topic

I have tested this code in 
* Ubuntu 20.04
* ROS noetic

## Build package
```
mkdir -p ~/catkin_ws/src
git clone {this/repository}
cd ..
source /opt/ros/{ROS_DISTRO}/setup.bash
mkdir build logs devel
catkin build unite_imu
```

After build, source this package by 
```
source devel/setup.bash
```

## Run node
To run unite_imu ros node, type the following in commandline
```
roslaunch unite_imu unite_imu.launch
```

This node subscribes to the following topics
* `/d435i/accel/sample`
* `/d435i/gyro/sample`

and publish the single imu topic
* `/d435i/imu`

If your sensor data are in different topic name, you may remap them in launch file.

## For ROS_DISTRO older than noetic
ROS melodic or older uses python2. 
But, by default, `src/unite_imu.py` uses python3. 
So, please change the first line of `#!/usr/bin/env python3` to `#!/usr/bin/env python`. 