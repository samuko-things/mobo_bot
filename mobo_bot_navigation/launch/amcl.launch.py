import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  # Set the path to this package.
  navigation_pkg_path = get_package_share_directory('mobo_bot_navigation')
  nav2_bringup_pkg_path = get_package_share_directory('nav2_bringup')
 
  # Set the path to the map file
  map_file_name = 'simple_world_map.yaml'
  map_yaml_path = os.path.join(navigation_pkg_path, 'map', map_file_name)

  # Set the path to the nav params file
  nav_params_file_name = 'nav2_bringup_params.yaml'
  nav_params_file = os.path.join(navigation_pkg_path, 'config', nav_params_file_name)
 
  #--------------------------------------------------------------------------

  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')
  map = LaunchConfiguration('map')
  params_file = LaunchConfiguration('params_file')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
  
  declare_map_cmd = DeclareLaunchArgument(
      name='map',
      default_value=map_yaml_path,
      description='file path to the map needed for navigation')
  
  declare_params_file_cmd = DeclareLaunchArgument(
      name='params_file',
      default_value=nav_params_file,
      description='file path to the navigation paramater file needed for navigation')

  #-----------------------------------------------------------------------------

  localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(nav2_bringup_pkg_path,'launch','localization_launch.py')]
            ), 
            launch_arguments={
              'map': map,
              'use_sim_time': use_sim_time,
              'params_file': params_file
            }.items()
  )

  #--------------------------------------------------------------------------------

  # Create the launch description
  ld = LaunchDescription()
 
  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_map_cmd)
  ld.add_action(declare_params_file_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(localization_launch)

  return ld
