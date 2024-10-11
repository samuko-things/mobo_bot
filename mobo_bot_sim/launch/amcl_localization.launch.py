import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            ExecuteProcess)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


 
def generate_launch_description():
  # Set the path to this package.
  my_description_pkg_path = get_package_share_directory('mobo_bot_description')
  my_rviz_pkg_path = get_package_share_directory('mobo_bot_rviz')
  my_sim_pkg_path = get_package_share_directory('mobo_bot_sim')
  my_nav_pkg_path = get_package_share_directory('mobo_bot_nav2d') 

  # robot name
  robot_name = 'mobo_bot'
  # initial robot pose
  x_pos = 0.0; y_pos = 0.0; yaw = 0.0

  # Set the path to the sim world file
  world_file_name = 'test_world.world'
  world_path = os.path.join(my_sim_pkg_path, 'world', world_file_name)

  # Set the path to the map file
  map_file_name = 'my_test_map.yaml'
  map_path = os.path.join(my_nav_pkg_path, 'maps', map_file_name)

  # Set the path to the nav param file
  nav_param_file_name = 'my_nav2_bringup_params.yaml'
  nav_param_path = os.path.join(my_nav_pkg_path, 'config', nav_param_file_name)

  # Set the path to the rviz file
  rviz_file_name = 'amcl_localization.rviz'
  rviz_path = os.path.join(my_rviz_pkg_path, 'config', rviz_file_name)


  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
  use_rviz = LaunchConfiguration('use_rviz')

  namespace = LaunchConfiguration('namespace')
  use_namespace = LaunchConfiguration('use_namespace')
  slam = LaunchConfiguration('slam')
  map_yaml_file = LaunchConfiguration('map_yaml_file')
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
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
  
  declare_use_rviz_cmd = DeclareLaunchArgument(
    'use_rviz',
    default_value= 'True',
    description='whether to run sim with rviz or not')


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
      'use_sim_time': use_sim_time,
      'yaml_filename': map_yaml_file}

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

  stdout_linebuf_envvar = SetEnvironmentVariable(
      'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

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
        default_value='False',
        description='Whether run a SLAM')

  declare_map_yaml_file_cmd = DeclareLaunchArgument(
      'map_yaml_file',
      default_value=map_path,
      description='Full path to map yaml file to load')

  declare_params_file_cmd = DeclareLaunchArgument(
      'params_file',
      default_value=nav_param_path,
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



  # Specify the actions
  start_gazebo_server_cmd = ExecuteProcess(
      condition=IfCondition(use_simulator),
      cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so', world],
      output='screen')

  start_gazebo_client_cmd = ExecuteProcess(
      condition=IfCondition(PythonExpression(
          [use_simulator, ' and not ', headless])),
      cmd=['gzclient'],
      output='screen')
 

  rsp_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(my_description_pkg_path,'launch','rsp.launch.py')]
            ), 
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_simulation': 'True'}.items()
  )

  rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
        condition=IfCondition(use_rviz)
  )

  # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
  spawn_entity_in_gazebo = Node(
      package='gazebo_ros', 
      executable='spawn_entity.py',
      arguments=[
          '-topic', '/robot_description',
          '-entity', robot_name,
          '-x', str(x_pos),
          '-y', str(y_pos),
          '-Y', str(yaw),
          ],
      output='screen')
  
  # navigation bringup
  nav_bringup_cmd_group = GroupAction([
      PushRosNamespace(
          condition=IfCondition(use_namespace),
          namespace=namespace
      ),

      Node(
          condition=IfCondition(use_composition),
          name='nav2_container',
          package='rclcpp_components',
          executable='component_container_isolated',
          parameters=[configured_params, {'autostart': autostart}],
          arguments=['--ros-args', '--log-level', log_level],
          remappings=remappings,
          output='screen'
      ),

      IncludeLaunchDescription(
          PythonLaunchDescriptionSource(os.path.join(my_nav_pkg_path, 'launch', 'amcl_localization.launch.py')),
          condition=IfCondition(PythonExpression(['not ', slam])),
          launch_arguments={'namespace': namespace,
                            'map_yaml_file': map_yaml_file,
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'params_file': params_file,
                            'use_composition': use_composition,
                            'use_respawn': use_respawn,
                            'container_name': 'nav2_container'}.items()
      )
  ])
  
  # Create the launch description
  ld = LaunchDescription()

  # Set environment variables
  ld.add_action(stdout_linebuf_envvar)

  # Declare the launch options
  ld.add_action(declare_headless_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
  ld.add_action(declare_use_rviz_cmd)

  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_slam_cmd)
  ld.add_action(declare_map_yaml_file_cmd)
  ld.add_action(declare_params_file_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_use_composition_cmd)
  ld.add_action(declare_use_respawn_cmd)
  ld.add_action(declare_log_level_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(rsp_launch)
  ld.add_action(rviz_node)
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_entity_in_gazebo)
  ld.add_action(nav_bringup_cmd_group)
 
  return ld