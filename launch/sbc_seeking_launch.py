from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
	light_dir = get_package_share_directory('light_sensor')
	publish_light = IncludeLaunchDescription(
        	PythonLaunchDescriptionSource(
                	[light_dir , '/light_launch.py']))

	bringup = IncludeLaunchDescription(
	        PythonLaunchDescriptionSource(
                [ThisLaunchFileDir() , '/my_tb3_bringup.launch.py']))


	return LaunchDescription([bringup,publish_light])
