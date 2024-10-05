import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition



def generate_launch_description():

    description_pkg_path = get_package_share_directory('mobo_bot_description')
    rviz_pkg_path = get_package_share_directory('mobo_bot_rviz')
    sim_pkg_path = get_package_share_directory('mobo_bot_sim')
    nav_pkg_path = get_package_share_directory('mobo_bot_nav2d')
    
    world_file_name = 'test_world.world'
    world_path = os.path.join(sim_pkg_path, 'world', world_file_name)

    # initialize launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    use_rviz_with_slam_map = LaunchConfiguration('use_rviz_with_slam_map')

    # declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='SDF world file'
    )

    declare_use_rviz_with_slam_map_cmd = DeclareLaunchArgument(
        'use_rviz_with_slam_map',
        default_value= 'True',
        description='whether to run slam mapping with rviz or not')


    sim_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(sim_pkg_path,'launch','sim.launch.py')]
            ), 
            launch_arguments={'use_sim_time': use_sim_time,
                              'world': world,
                              'use_rviz_with_sim': 'False'}.items()
    )

    rviz_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(rviz_pkg_path,'launch','build_map_with_slam.launch.py')]),
      condition=IfCondition(use_rviz_with_slam_map)
    )
    
    
    slam_mapping_param_file = os.path.join(nav_pkg_path,'config','my_slam_mapping_params_online_async.yaml')
    slam_mapping_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_mapping_param_file],
    )





    # Create the launch description and populate
    ld = LaunchDescription()


    # add the necessary declared launch arguments to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_rviz_with_slam_map_cmd)
    
    # Add the nodes to the launch description
    ld.add_action(sim_robot)
    ld.add_action(rviz_launch)
    ld.add_action(slam_mapping_node)

    return ld




    