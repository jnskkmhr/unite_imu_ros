import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unite_imu',
            executable='unite_imu',
            name='unite_imu',
            output='screen',
            # remappings=[()], # you may remap topics here
        ), 
    ])