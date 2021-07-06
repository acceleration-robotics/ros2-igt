import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	pkg_igt_nav = get_package_share_directory('igt_nav')
	pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

	use_sim_time = LaunchConfiguration('use_sim_time')
	map_yaml_file = LaunchConfiguration('map',
					default=os.path.join(pkg_igt_nav, 'maps', 'lab_map.yaml'))
	nav2_config_file = LaunchConfiguration('params', 
						default=os.path.join(pkg_igt_nav, 'config', 'nav2.yaml'))

	# nav2 bringup launch file
	nav2_bringup_launch_file = IncludeLaunchDescription(
					PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
					launch_arguments={
						'map': map_yaml_file,
						'use_sim_time': use_sim_time,
						'params_file': nav2_config_file}.items(),
				)	
	
	return LaunchDescription([
		DeclareLaunchArgument('map',
					default_value=map_yaml_file,
					description="Path to Map yaml file"),
		DeclareLaunchArgument('params_file',
		     		default_value=nav2_config_file,
		     		description="Path to nav2 config file"),
		DeclareLaunchArgument('use_sim_time',
                                default_value='true',
                                description="Use sim clock or not"),
		nav2_bringup_launch_file
		])
