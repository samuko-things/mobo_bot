import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():
  # Set the path to this package.
  my_description_pkg_path = get_package_share_directory('mobo_bot_description')
  my_rviz_pkg_path = get_package_share_directory('mobo_bot_rviz')
  my_sim_pkg_path = get_package_share_directory('mobo_bot_sim') 

  # robot name
  robot_name = 'mobo_bot'
  # initial robot pose
  x_pos = 0.0; y_pos = 0.0; yaw = 0.0

  # Set the path to the world file
  world_file_name = 'new_test_world.world'
  world_path = os.path.join(my_sim_pkg_path, 'world', world_file_name)

  # Set rviz config file
  rviz_config_file = os.path.join(my_rviz_pkg_path,'config','sim.rviz')
 
 
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
  use_rviz = LaunchConfiguration('use_rviz')
 
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
        arguments=['-d', rviz_config_file],
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
  
  # Create the launch description
  ld = LaunchDescription()
 
  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_headless_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
  ld.add_action(declare_use_rviz_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(rsp_launch)
  ld.add_action(rviz_node)
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_entity_in_gazebo)
 
  return ld