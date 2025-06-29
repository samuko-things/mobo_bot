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
  navigation_pkg_path = get_package_share_directory('mobo_bot_nav_test')
  nav2_bringup_pkg_path = get_package_share_directory('nav2_bringup')
 
  # Set the path to the map file
  map_file_name = 'room_with_walls.yaml'
  map_yaml_path = os.path.join(navigation_pkg_path, 'maps', map_file_name)

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

  # a_star_planner_node = Node(
  #   package='mobo_bot_nav_test',
  #   executable='a_star_planner.py',
  #   name='a_star_planner',
  #   output='screen',
  # )

  # pure_pursuit_node = Node(
  #   package='mobo_bot_nav_test',
  #   executable='pure_pursuit',
  #   name='pure_pursuit',
  #   output='screen',
  #   parameters=[{'look_ahead_distance': 0.5,
  #                'max_linear_velocity': 0.3,
  #                'max_angular_velocity': 1.0
  #                }],
  # )

  lifecycle_nodes = ["map_server", "amcl", "costmap", "planner_server", "controller_server", "bt_navigator", "behavior_server"]
  remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

  nav2_map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[params_file, {'yaml_filename': map}],
    remappings=remappings,
  )

  nav2_costmap_2d_node = Node(
    package='nav2_costmap_2d',
    executable='nav2_costmap_2d',
    name='costmap',
    output='screen',
    parameters=[params_file],
  )

  nav2_amcl_node = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    output='screen',
    parameters=[params_file],
    remappings=remappings,
  )

  nav2_planner_server_node = Node(
    package='nav2_planner',
    executable='planner_server',
    name='planner_server',
    output='screen',
    parameters=[params_file],
    remappings=remappings,
  )

  nav2_controller_server_node = Node(
    package='nav2_controller',
    executable='controller_server',
    name='controller_server',
    output='screen',
    parameters=[params_file],
    remappings=remappings,
  )

  nav2_bt_navigator_node = Node(
    package='nav2_bt_navigator',
    executable='bt_navigator',
    name='bt_navigator',
    output='screen',
    parameters=[params_file],
    remappings=remappings,
  )

  nav2_behavior_server_node = Node(
    package='nav2_behaviors',
    executable='behavior_server',
    name='behavior_server',
    output='screen',
    parameters=[params_file],
    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
  )

  nav2_lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    output='screen',
    parameters=[{"autostart": True, "bond_timeout": 0.0}, {'node_names': lifecycle_nodes}],
  )

  #--------------------------------------------------------------------------------

  # Create the launch description
  ld = LaunchDescription()
 
  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_map_cmd)
  ld.add_action(declare_params_file_cmd)
 
  # Add the nodes to the launch description
  # ld.add_action(a_star_planner_node)
  # ld.add_action(pure_pursuit_node)
  ld.add_action(nav2_map_server_node)
  ld.add_action(nav2_costmap_2d_node)
  ld.add_action(nav2_amcl_node)
  ld.add_action(nav2_planner_server_node)
  ld.add_action(nav2_controller_server_node)
  ld.add_action(nav2_bt_navigator_node)
  ld.add_action(nav2_behavior_server_node)
  ld.add_action(nav2_lifecycle_manager_node)

  return ld
