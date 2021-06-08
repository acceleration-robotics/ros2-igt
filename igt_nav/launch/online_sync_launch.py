import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
	pkg_igt_nav = get_package_share_directory('igt_nav')

	return LaunchDescription([
		DeclareLaunchArgument('use_sim_time',
					default_value
	])
