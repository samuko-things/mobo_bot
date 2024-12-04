import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  ExecuteProcess,
  IncludeLaunchDescription,
  RegisterEventHandler,
  SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
 
def generate_launch_description():
  # Set the path to this package.
  my_sim_pkg_path = get_package_share_directory('mobo_bot_sim') 


  # Launch configuration variables specific to simulation
  use_ekf = LaunchConfiguration('use_ekf')
  odom_topic = LaunchConfiguration('odom_topic')

  
  declare_use_ekf_cmd = DeclareLaunchArgument(
      name='use_ekf',
      default_value='True',
      # default_value='False',
      description='fuse odometry and imu data if true')
  
  declare_odom_topic_cmd = DeclareLaunchArgument(
      name='odom_topic',
      default_value='odom',
      description='topic to remap /odometry/filtered to')
  #------------------------------------------------------------

  
  # Localize using odometry and IMU data. 
  # It can be turned off because the navigation stack uses AMCL with lidar data for localization
  ekf_config_path = os.path.join(my_sim_pkg_path,'config','ekf.yaml')
  ekf_node = Node(
          package='robot_localization',
          executable='ekf_node',
          name='ekf_filter_node',
          output='screen',
          parameters=[
              ekf_config_path,
          ],
          condition=IfCondition(use_ekf),
          remappings=[("odometry/filtered", odom_topic)]
      )
  
  # Create the launch description
  ld = LaunchDescription()

  ld.add_action(declare_use_ekf_cmd)
  ld.add_action(declare_odom_topic_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(ekf_node)

 
  return ld