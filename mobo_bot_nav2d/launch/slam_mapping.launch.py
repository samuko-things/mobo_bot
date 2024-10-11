import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition



def generate_launch_description():

    my_nav_pkg_path = get_package_share_directory('mobo_bot_nav2d')

    # Set the path to the nav param file
    slam_mapping_param_file_name = 'my_slam_mapping_params_online_async.yaml'
    slam_mapping_param_path = os.path.join(my_nav_pkg_path, 'config', slam_mapping_param_file_name)

    params_file = LaunchConfiguration('params_file')

    declare_params_file_cmd = DeclareLaunchArgument(
      'params_file',
      default_value=slam_mapping_param_path,
      description='Full path to the ROS2 navigation parameters file to use for all launched nodes')
        
    slam_mapping_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file],
    )
    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)

    # add the necessary declared launch arguments to the launch description
    ld.add_action(slam_mapping_node)

    return ld




    