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
  base_pkg_path = get_package_share_directory('mobo_bot_base')
  nav2_bringup_pkg_path = get_package_share_directory('nav2_bringup')

  # Set the path to the nav param file
  nav_param_file_name = 'nav2_bringup_params.yaml'
  nav_param_file_path = os.path.join(base_pkg_path, 'config', nav_param_file_name)

  map_file_name = 'my_map.yaml'
  map_yaml_path = os.path.join(base_pkg_path, 'map', map_file_name)


  #----------------------------------------------------------------------------------

  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_ekf = LaunchConfiguration('use_ekf')
  use_lidar = LaunchConfiguration('use_lidar')
  launch_robot = LaunchConfiguration('launch_robot')
 
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true')
  
  declare_launch_robot_cmd = DeclareLaunchArgument(
    'launch_robot',
    default_value= 'False',
    description='whether to run simulation or not')
  
  declare_use_ekf_cmd = DeclareLaunchArgument(
      name='use_ekf',
      default_value='False',
      description='fuse odometry and imu data if true')
  
  declare_use_lidar_cmd = DeclareLaunchArgument(
      name='use_lidar',
      default_value='True',
      description='use rplidar if true')

  
  robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(base_pkg_path,'launch','robot.launch.py')]
            ), 
            launch_arguments={
              'use_lidar': use_lidar,
              'use_ekf': use_ekf,
            }.items(),
            condition=IfCondition(launch_robot)
  )

  #-------------------------------------------------------------------------------


  #-----------------------------------------------------------------------------

  slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(nav2_bringup_pkg_path,'launch','slam_launch.py')]
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
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_ekf_cmd)
  ld.add_action(declare_use_lidar_cmd)
  ld.add_action(declare_launch_robot_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(robot_launch)
  ld.add_action(slam_launch)
 
  return ld