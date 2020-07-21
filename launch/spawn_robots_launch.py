import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    model_file_name = 'source_turtlebot.sdf'

    model = os.path.join(get_package_share_directory('fim_track_2'), 'models', model_file_name)

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    return LaunchDescription([       
        # To do: spawn sdf models from customized urdf.
        Node(  package = 'fim_track_2',
                node_namespace = 'tb1',
                node_executable = 'spawn_entitiy',
                node_name = 'tk',
                arguments = ['-file',model,'-entity','tk']
            ),
        # To do: create robot state publisher.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
