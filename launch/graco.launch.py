from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os.path
import yaml
from launch.substitutions import TextSubstitution, LaunchConfiguration, Command

def generate_launch_description():

	ld = LaunchDescription()

	bag_path = '/root/ground_01'
	# ns_mappings = {
    #  'Alpha': 'robot_0',
    #  'Bob': 'robot_1',
    #  'Carol': 'robot_2',
    #  }

	## rosbag play
	robot_0_bag_node = ExecuteProcess(
		cmd=[
			'ros2', 'bag', 'play',
			bag_path,
			'--remap',
			'/Alpha/velodyne_points:=' + ns_mappings['Alpha'] + '/velodyne_points',
			'/Alpha/imu/data:=' + ns_mappings['Alpha'] + '/imu/data',
			'/Alpha/heading:=' + ns_mappings['Alpha'] + '/heading',
			'/Alpha/correct_imu:=' + ns_mappings['Alpha'] + '/correct_imu',
			'/Alpha/fix:=' + ns_mappings['Alpha'] + '/fix',
			'/Alpha/time_reference:=' + ns_mappings['Alpha'] + '/time_reference',
			'/Alpha/vel:=' + ns_mappings['Alpha'] + '/vel',
			'/Alpha/nlink_linktrack_nodeframe2:=' + ns_mappings['Alpha'] + '/nlink_linktrack_nodeframe2',
   
			'/Bob/velodyne_points:=' + ns_mappings['Bob'] + '/velodyne_points',
			'/Bob/imu/data:=' + ns_mappings['Bob'] + '/imu/data',
			'/Bob/heading:=' + ns_mappings['Bob'] + '/heading',
			'/Bob/correct_imu:=' + ns_mappings['Bob'] + '/correct_imu',
			'/Bob/fix:=' + ns_mappings['Bob'] + '/fix',
			'/Bob/time_reference:=' + ns_mappings['Bob'] + '/time_reference',
			'/Bob/vel:=' + ns_mappings['Bob'] + '/vel',
			'/Bob/nlink_linktrack_nodeframe2:=' + ns_mappings['Bob'] + '/nlink_linktrack_nodeframe2',
   
			'/Carol/velodyne_points:=' + ns_mappings['Carol'] + '/velodyne_points',
			'/Carol/imu/data:=' + ns_mappings['Carol'] + '/imu/data',
			'/Carol/heading:=' + ns_mappings['Carol'] + '/heading',
			'/Carol/correct_imu:=' + ns_mappings['Carol'] + '/correct_imu',
			'/Carol/fix:=' + ns_mappings['Carol'] + '/fix',
			'/Carol/time_reference:=' + ns_mappings['Carol'] + '/time_reference',
			'/Carol/vel:=' + ns_mappings['Carol'] + '/vel',
			'/Carol/nlink_linktrack_nodeframe2:=' + ns_mappings['Carol'] + '/nlink_linktrack_nodeframe2',
		],
		output='screen',
	)
	ld.add_action(robot_0_bag_node)

	## rosbag play
	# robot_1_bag_node = ExecuteProcess(
	# 	cmd=[
	# 		'ros2', 'bag', 'play',
	# 		r1_bag_path,
	# 		'--remap',
	# 		'/velodyne_points:=' + r1_name + '/velodyne_points',
	# 		'/fix:=' + r1_name + '/fix',
	# 		'/camera/imu:=' + r1_name + '/camera/imu',
	# 		'/camera/color/image_raw:=' + r1_name + '/camera/color/image_raw',
	# 	],
	# 	name='bag_r1',
	# 	output='screen',
	# )
	# ld.add_action(robot_1_bag_node)

	# centrailized rviz
	# rviz_config_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'rviz2_kitech.rviz')
	# rviz_node = Node(
	# 	package = 'rviz2',
	# 	namespace = '',
	# 	executable = 'rviz2',
	# 	name = 'co_lrio_rviz',
	# 	respawn=True, 
	# 	arguments = ['-d' + rviz_config_path]
	# )
	# ld.add_action(rviz_node)

	return ld
