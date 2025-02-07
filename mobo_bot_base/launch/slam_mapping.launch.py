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
  base_pkg_path = get_package_share_directory('mobo_bot_base')

  # Set the path to the nav param file
  slam_mapping_param_file_name = 'slam_mapping_params_online_async.yaml'
  slam_mapping_param_file_path = os.path.join(base_pkg_path, 'config', slam_mapping_param_file_name)


  #----------------------------------------------------------------------------------

  # Launch configuration variables specific to simulation
  # use_sim_time = LaunchConfiguration('use_sim_time')
  launch_robot = LaunchConfiguration('launch_robot')
  params_file = LaunchConfiguration('params_file')
 
     
  # declare_use_sim_time_cmd = DeclareLaunchArgument(
  #   name='use_sim_time',
  #   default_value='False',
  #   description='Use simulation (Gazebo) clock if true')
  
  declare_launch_robot_cmd = DeclareLaunchArgument(
    'launch_robot',
    default_value= 'True',
    description='whether to run robot or not')
  
  robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(base_pkg_path,'launch','robot.launch.py')]
            ),
            condition=IfCondition(launch_robot)
  )

  #-------------------------------------------------------------------------------


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
  # ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_launch_robot_cmd)
  ld.add_action(declare_params_file_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(robot_launch)
  ld.add_action(slam_mapping_node)
 
  return ld