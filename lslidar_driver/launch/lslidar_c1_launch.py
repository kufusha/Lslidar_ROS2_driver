#!/usr/bin/python3
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
import lifecycle_msgs.msg


def generate_launch_description():
    driver_dir = os.path.join(
        get_package_share_directory('lslidar_driver'), 'params', 'lslidar_c1.yaml'
    )
    rviz_dir = os.path.join(
        get_package_share_directory('lslidar_driver'), 'rviz_cfg', 'lslidar_c1.rviz'
    )

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    driver_node = ""
    rviz_node = ""
    ros_version = p.communicate()[0]
    print(ros_version)
    if ros_version == b'dashing\n' or ros_version == b'eloquent\n':
        print("ROS VERSION: dashing/eloquent")
        driver_node = LifecycleNode(
            package='lslidar_driver',
            node_namespace='c1',
            node_executable='lslidar_driver_node',
            node_name='lslidar_driver_node',
            output='screen',
            parameters=[driver_dir],
        )
        rviz_node = Node(
            package='rviz2',
            node_namespace='c1',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_dir],
            output='screen',
        )
    elif ros_version == b'foxy\n' or ros_version == b'galactic\n' or ros_version == b'humble\n':
        print("ROS VERSION: foxy/galactic/humble")
        driver_node = LifecycleNode(
            package='lslidar_driver',
            namespace='c1',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[driver_dir],
        )
        rviz_node = Node(
            package='rviz2',
            namespace='c1',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_dir],
            output='screen',
        )

    else:
        print("Please configure the ros environment")
        exit()

    return LaunchDescription([driver_node, rviz_node])
