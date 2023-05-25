# ROS2 node for unite-imu method
This is ROS2 node for 
* interpolating accel message to the gyro's timestamp
* fusing interpolated accel and gyro value into single imu topic

I have tested this code in 
* Ubuntu 20.04
* ROS2 foxy

## Build package
```
mkdir -p ~/ros2_ws/src
git clone {this/repository}
cd ..
source /opt/ros/{ROS_DISTRO}/setup.bash
colcon buid --symlink-install --packages-select unite_imu
```

After build, source this package by 
```
source install/setup.bash
```

## Run node
To run unite_imu ros node, type the following in commandline
```
ros2 launch unite_imu unite_imu.launch.py
```

This node subscribes to the following topics
* `/d435i/accel/sample`
* `/d435i/gyro/sample`

and publish the single imu topic
* `/d435i/imu`

If your sensor data are in different topic name, you may remap them in launch file.