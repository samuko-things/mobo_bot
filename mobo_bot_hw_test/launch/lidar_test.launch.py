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


  #----------------------------------------------------------------------------------

  
  robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(base_pkg_path,'launch','robot.launch.py')]
            ), 
            launch_arguments={
              'use_epmc': 'False',
              'use_eimu': 'False',
              'use_lidar': 'True',
              'use_camera': 'False',
              'use_ekf': 'False',
            }.items(),
  )

  #-------------------------------------------------------------------------------


  # Create the launch description
  ld = LaunchDescription()
 
  # Add the nodes to the launch description
  ld.add_action(robot_launch)
 
  return ld