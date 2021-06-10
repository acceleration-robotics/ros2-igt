import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	pkg_igt_nav = get_package_share_directory('igt_nav')

	use_sim_time = LaunchConfiguration('use_sim_time')
	slam_params_file = LaunchConfiguration('slam_params_file')

	declare_use_sim_time = DeclareLaunchArgument('use_sim_time',
				default_value = 'true',
				description = 'Use simulation/Gazebo clock')

	declare_slam_params_file = DeclareLaunchArgument('slam_params_file',
				default_value = os.path.join(pkg_igt_nav, 'config', 'mapper_params_online_sync.yaml'),
				description = 'Path to ROS param file')

	# sync slam toolbox node
	start_sync_slam_toolbox = Node(package = 'slam_toolbox', 
					executable = 'sync_slam_toolbox_node',
					name = 'slam_toolbox',
					output = 'screen',
					parameters = [
						slam_params_file,
						{'use_sim_time': use_sim_time}])

	return LaunchDescription([
		declare_use_sim_time,
		declare_slam_params_file,
		start_sync_slam_toolbox
	])
