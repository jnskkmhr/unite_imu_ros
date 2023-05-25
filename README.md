# ROS/ROS2 node for unite-imu method

This is ROS1/ROS2 node for 
* interpolating accel message to the gyro's timestamp
* fusing interpolated accel and gyro value into single imu topic

It basically behaves same as `linear interpolation` mode in `unite_imu` method in Realsense ROS package.

For example, D400 cameras have bult in IMU components which produce 2 streams : `gyro (angular velocity)` and `accel (linear acceleration)` each with its own frequency. (gyro is in higher frequency than accel.)

In some cases, you may need single "united" imu topic since some open source software require IMU streams in a single sensor_msgs::Imu. 

In this way, this repository is helpful in returning united imu topic instead of separate gyro and accel streams. 

I have tested this code in 
* Ubuntu 20.04
* ROS noetic/ROS2 foxy

For each package for ROS1 and ROS2, please refer to directory `ros1_ws` and `ros2_ws`. 
