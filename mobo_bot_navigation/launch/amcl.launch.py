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

  # Set the path to the nav param file
  nav_param_file_name = 'nav2_bringup_params.yaml'
  nav_param_file_path = os.path.join(navigation_pkg_path, 'config', nav_param_file_name)
 
  #--------------------------------------------------------------------------

  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')
 
  declare_headless_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to run only gzserver')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

  #-----------------------------------------------------------------------------

  localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(nav2_bringup_pkg_path,'launch','localization_launch.py')]
            ), 
            launch_arguments={
              'map': map_yaml_path,
              'use_sim_time': use_sim_time,
              'params_file': nav_param_file_path
            }.items()
  )

  #--------------------------------------------------------------------------------

  # Create the launch description
  ld = LaunchDescription()
 
  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_headless_cmd)
  ld.add_action(declare_use_sim_time_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(localization_launch)

  return ld
