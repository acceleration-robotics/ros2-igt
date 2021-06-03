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
	robot_name = 'igt_one'
	ign_model_prefix = '/model/' + robot_name

	# cmd_vel bridge 
	cmd_vel_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
			namespace = namespace,
			name = 'cmd_vel_bridge',
			output='screen',
			arguments = [
				ign_model_prefix + '/cmd_vel' + '@geometry_msgs/msg/Twist' + ']ignition.msgs.Twist'
			],
			remappings = [
				(ign_model_prefix + '/cmd_vel', '/cmd_vel')
			])

	# odometry bridge 
	odometry_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
			namespace = namespace,
			name = 'odometry_bridge',
			output='screen',
			arguments = [
				 ign_model_prefix + '/odometry' + '@nav_msgs/msg/Odometry' + '[ignition.msgs.Odometry'
			],
			remappings = [
				(ign_model_prefix + '/odometry', '/odom_raw')
			])

	# joint state bridge 
	joint_state_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
			namespace = namespace,
			name = 'joint_state_bridge',
			output='screen',
			arguments = [
				'/world/default/model/igt_one/joint_state'
				 + '@sensor_msgs/msg/JointState' + '[ignition.msgs.Model'
			],
			remappings = [
				('/world/default/model/igt_one/joint_state', '/joint_states')
			])

	return LaunchDescription([
		cmd_vel_bridge,
		joint_state_bridge,
		odometry_bridge
	])
