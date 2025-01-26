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
  rviz_pkg_path = get_package_share_directory('mobo_bot_rviz')
  sim_pkg_path = get_package_share_directory('mobo_bot_sim')

  # Set the path to the world file
  world_file_name = 'empty.sdf'
  world_file_path = os.path.join(sim_pkg_path, 'world', world_file_name)

  # Set rviz config file
  rviz_file_name = 'slam_mapping.rviz'
  rviz_file_path = os.path.join(rviz_pkg_path, 'config', rviz_file_name)
 
  # Set the path to the nav param file
  slam_mapping_param_file_name = 'slam_mapping_params_online_async.yaml'
  slam_mapping_param_file_path = os.path.join(sim_pkg_path, 'config', slam_mapping_param_file_name)
 

  #--------------------------------------------------------------------------
  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  world_path = LaunchConfiguration('world_path')
  rviz_path = LaunchConfiguration('rviz_path')
  use_rviz = LaunchConfiguration('use_rviz')
  use_ekf = LaunchConfiguration('use_ekf')
  launch_sim = LaunchConfiguration('launch_sim')
  params_file = LaunchConfiguration('params_file')
 
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
  
  declare_use_rviz_cmd = DeclareLaunchArgument(
    'use_rviz',
    default_value= 'True',
    description='whether to run sim with rviz or not')
  
  declare_use_ekf_cmd = DeclareLaunchArgument(
      name='use_ekf',
      default_value='True',
      # default_value='False',
      description='fuse odometry and imu data if true')
  
  declare_launch_sim_cmd = DeclareLaunchArgument(
    'launch_sim',
    default_value= 'True',
    description='whether to run simulation or not')

  
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

  declare_params_file_cmd = DeclareLaunchArgument(
      'params_file',
      default_value=slam_mapping_param_file_path,
      description='Full path to the ROS2 navigation parameters file to use for all launched nodes')
  
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
  ld.add_action(declare_headless_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_world_path_cmd)
  ld.add_action(declare_rviz_path_cmd)
  ld.add_action(declare_use_rviz_cmd)
  ld.add_action(declare_use_ekf_cmd)
  ld.add_action(declare_params_file_cmd)
  ld.add_action(declare_launch_sim_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(sim_launch)
  ld.add_action(slam_mapping_node)

 
  return ld