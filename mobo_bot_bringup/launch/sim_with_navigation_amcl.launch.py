import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():
  # Set the path to this package.
  sim_pkg_path = get_package_share_directory('mobo_bot_sim')
  rviz_pkg_path = get_package_share_directory('mobo_bot_rviz')
  navigation_pkg_path = get_package_share_directory('mobo_bot_navigation')
 
  #--------------------------------------------------------------------------

  # Launch configuration variables specific to simulation
  world_name = LaunchConfiguration('world_name')
  map_name = LaunchConfiguration('map_name')
  param_name = LaunchConfiguration('param_name')
 
  declare_world_name_cmd = DeclareLaunchArgument(
    name='world_name',
    default_value='empty',
    description='name of the world file')
  
  world_path = PathJoinSubstitution([
          sim_pkg_path,
          "world",
          PythonExpression(expression=["'", world_name, "'", " + '.sdf'"])
      ]
  )

  declare_map_name_cmd = DeclareLaunchArgument(
    name='map_name',
    default_value='simple_world_map',
    description='name of the map file')
  
  map_path = PathJoinSubstitution([
          navigation_pkg_path,
          "map",
          PythonExpression(expression=["'", map_name, "'", " + '.yaml'"])
      ]
  )

  declare_param_name_cmd = DeclareLaunchArgument(
    name='param_name',
    default_value='nav2_bringup_params',
    description='name of the navigation parameter file')
  
  param_path = PathJoinSubstitution([
          navigation_pkg_path,
          "config",
          PythonExpression(expression=["'", param_name, "'", " + '.yaml'"])
      ]
  )

  #-----------------------------------------------------------------------------
  sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(sim_pkg_path,'launch','sim.launch.py')]
            ), 
            launch_arguments={
              'use_sim_time': 'true',
              'headless': 'false',
              'world_path': world_path,
            }.items(),
  )

  rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(rviz_pkg_path,'launch','nav_bringup.launch.py')]
            )
  )

  nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(navigation_pkg_path,'launch','nav_bringup.launch.py')]
            ), 
            launch_arguments={
              'slam': 'false',
              'map_file_path': map_path,
              'use_sim_time': 'true',
              'params_file_path': param_path
            }.items()
  )

  #--------------------------------------------------------------------------------

  # Create the launch description
  ld = LaunchDescription()
 
  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_world_name_cmd)
  ld.add_action(declare_map_name_cmd)
  ld.add_action(declare_param_name_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(sim_launch)
  ld.add_action(rviz_launch)
  ld.add_action(nav_launch)

  return ld
