import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
def generate_launch_description():
  # Set the path to this package.
  navigation_pkg_path = get_package_share_directory('mobo_bot_navigation')
 
  # Set the path to the nav param file
  slam_mapping_param_file_name = 'slam_mapping_params_online_async.yaml'
  slam_mapping_param_file_path = os.path.join(navigation_pkg_path, 'config', slam_mapping_param_file_name)
 

  #--------------------------------------------------------------------------
  
  # Launch configuration variables specific to simulation
  params_file = LaunchConfiguration('params_file')

  declare_params_file_cmd = DeclareLaunchArgument(
      'params_file',
      default_value=slam_mapping_param_file_path,
      description='Full path to the ROS2 navigation parameters file to use for all launched nodes')

  #-----------------------------------------------------------------------------
  
  slam_mapping_node = Node(
      package='slam_toolbox',
      executable='async_slam_toolbox_node',
      name='slam_toolbox',
      output='screen',
      parameters=[params_file],
    )

  #--------------------------------------------------------------------------------

  # Create the launch description
  ld = LaunchDescription()
 
  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_params_file_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(slam_mapping_node)
 
  return ld