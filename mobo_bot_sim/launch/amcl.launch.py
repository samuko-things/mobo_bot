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
  rviz_pkg_path = get_package_share_directory('mobo_bot_rviz')
  sim_pkg_path = get_package_share_directory('mobo_bot_sim')
  nav2_bringup_pkg_path = get_package_share_directory('nav2_bringup')

  # Set the path to the world file
  world_file_name = 'empty.sdf'
  world_file_path = os.path.join(sim_pkg_path, 'world', world_file_name)

  # Set rviz config file
  rviz_file_name = 'amcl.rviz'
  rviz_file_path = os.path.join(rviz_pkg_path, 'config', rviz_file_name)
 
  # Set the path to the map file
  map_file_name = 'simple_world_map.yaml'
  map_yaml_path = os.path.join(sim_pkg_path, 'map', map_file_name)

  # Set the path to the nav param file
  nav_param_file_name = 'nav2_bringup_params.yaml'
  nav_param_file_path = os.path.join(sim_pkg_path, 'config', nav_param_file_name)
 

  #--------------------------------------------------------------------------

  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  world_path = LaunchConfiguration('world_path')
  rviz_path = LaunchConfiguration('rviz_path')
  use_rviz = LaunchConfiguration('use_rviz')
  use_ekf = LaunchConfiguration('use_ekf')
  launch_sim = LaunchConfiguration('launch_sim')

 
  declare_headless_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to run only gzserver')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
 
  declare_world_path_cmd = DeclareLaunchArgument(
    name='world_path',
    default_value=world_file_path,
    description='Full path to the world model file to load')
  
  declare_rviz_path_cmd = DeclareLaunchArgument(
    name='rviz_path',
    default_value=rviz_file_path,
    description='Full path to the world model file to load')
  
  declare_launch_sim_cmd = DeclareLaunchArgument(
    'launch_sim',
    default_value= 'True',
    description='whether to run simulation or not')
  
  declare_use_rviz_cmd = DeclareLaunchArgument(
    'use_rviz',
    default_value= 'True',
    description='whether to run sim with rviz or not')
  
  declare_use_ekf_cmd = DeclareLaunchArgument(
      name='use_ekf',
      default_value='True',
      # default_value='False',
      description='fuse odometry and imu data if true')
  
  
  
  sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(sim_pkg_path,'launch','sim.launch.py')]
            ), 
            launch_arguments={
              'use_sim_time': use_sim_time,
              'headless': headless,
              'world_path': world_path,
              'use_rviz': use_rviz,
              'rviz_path': rviz_path,
              'use_ekf': use_ekf,
            }.items(),
            condition=IfCondition(launch_sim)
  )

  #-----------------------------------------------------------------------------



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
  ld.add_action(declare_world_path_cmd)
  ld.add_action(declare_rviz_path_cmd)
  ld.add_action(declare_use_rviz_cmd)
  ld.add_action(declare_use_ekf_cmd)
  ld.add_action(declare_launch_sim_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(sim_launch)
  ld.add_action(localization_launch)

 
  return ld