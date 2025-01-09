import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  GroupAction,
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
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString
 
def generate_launch_description():
  # Set the path to this package.
  my_rviz_pkg_path = get_package_share_directory('mobo_bot_rviz')
  my_sim_pkg_path = get_package_share_directory('mobo_bot_sim')
  my_nav_pkg_path = get_package_share_directory('mobo_bot_nav2d') 

  # Set the path to the world file
  world_file_name = 'empty.sdf'
  world_file_path = os.path.join(my_sim_pkg_path, 'world', world_file_name)

  # Set rviz config file
  rviz_file_name = 'slam_mapping.rviz'
  rviz_file_path = os.path.join(my_rviz_pkg_path, 'config', rviz_file_name)

  # Set the path to the nav param file
  nav_param_file_name = 'my_nav2_bringup_sim_params.yaml'
  nav_param_file_path = os.path.join(my_nav_pkg_path, 'config', nav_param_file_name)
 

  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  world_path = LaunchConfiguration('world_path')
  rviz_path = LaunchConfiguration('rviz_path')
  use_rviz = LaunchConfiguration('use_rviz')
  use_ekf = LaunchConfiguration('use_ekf')
  launch_sim = LaunchConfiguration('launch_sim')

  namespace = LaunchConfiguration('namespace')
  use_namespace = LaunchConfiguration('use_namespace')
  slam = LaunchConfiguration('slam')
  params_file = LaunchConfiguration('params_file')
  autostart = LaunchConfiguration('autostart')
  use_composition = LaunchConfiguration('use_composition')
  use_respawn = LaunchConfiguration('use_respawn')
  log_level = LaunchConfiguration('log_level')
 
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
  
  

  # Map fully qualified names to relative ones so the node's namespace can be prepended.
  # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
  # https://github.com/ros/geometry2/issues/32
  # https://github.com/ros/robot_state_publisher/pull/30
  # TODO(orduno) Substitute with `PushNodeRemapping`
  #              https://github.com/ros2/launch_ros/issues/56
  remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

  # Create our own temporary YAML files that include substitutions
  param_substitutions = {
        'use_sim_time': use_sim_time}

  # Only it applys when `use_namespace` is True.
  # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
  # in config file 'nav2_multirobot_params.yaml' as a default & example.
  # User defined config file should contain '<robot_namespace>' keyword for the replacements.
  params_file = ReplaceString(
      source_file=params_file,
      replacements={'mobo_bot': ('/', namespace)},
      condition=IfCondition(use_namespace))

  configured_params = ParameterFile(
      RewrittenYaml(
          source_file=params_file,
          root_key=namespace,
          param_rewrites=param_substitutions,
          convert_types=True),
      allow_substs=True)

  declare_namespace_cmd = DeclareLaunchArgument(
      'namespace',
      default_value='',
      description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
      'use_namespace',
      default_value='false',
      description='Whether to apply a namespace to the navigation stack')
  
  declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether run a SLAM')

  declare_params_file_cmd = DeclareLaunchArgument(
      'params_file',
      default_value=nav_param_file_path,
      description='Full path to the ROS2 navigation parameters file to use for all launched nodes')

  declare_autostart_cmd = DeclareLaunchArgument(
      'autostart', default_value='true',
      description='Automatically startup the nav2 stack')

  declare_use_composition_cmd = DeclareLaunchArgument(
      'use_composition', default_value='True',
      description='Whether to use composed bringup')

  declare_use_respawn_cmd = DeclareLaunchArgument(
      'use_respawn', default_value='False',
      description='Whether to respawn if a node crashes. Applied when composition is disabled.')

  declare_log_level_cmd = DeclareLaunchArgument(
      'log_level', default_value='info',
      description='log level')
  #------------------------------------------------------------


  
  sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(my_sim_pkg_path,'launch','sim.launch.py')]
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

  # navigation bringup
  nav_bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(my_nav_pkg_path, 'launch', 'slam.launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'params_file': params_file}.items()),

    ])


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
  
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_slam_cmd)
  ld.add_action(declare_params_file_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_use_composition_cmd)
  ld.add_action(declare_use_respawn_cmd)
  ld.add_action(declare_log_level_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(sim_launch)
  ld.add_action(nav_bringup_cmd_group)

 
  return ld