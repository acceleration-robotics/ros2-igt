import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

	pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
	pkg_igt_ignition = get_package_share_directory('igt_ignition')

	namespace = 'igt_one'

	# teleop bridge 
	teleop_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
			namespace = namespace,
			name = 'teleop_bridge',
			output='screen',
			arguments = [
				'/cmd_vel' + '@geometry_msgs/msg/Twist' + '@ignition.msgs.Twist'
			])

	return LaunchDescription([
		teleop_bridge
	])
