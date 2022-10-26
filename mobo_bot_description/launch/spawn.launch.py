import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro



def generate_launch_description():

    pkg_path = get_package_share_directory('mobo_bot_description')

    xacro_file = os.path.join(pkg_path,'urdf','robot_urdf_description.xacro')
    robot_description_doc = xacro.parse(open(xacro_file))
    xacro.process_doc(robot_description_doc)

    # world_file_name = 'race_track.world'
    # world_path = os.path.join(pkg_path, 'world', world_file_name)



    params = {'robot_description': robot_description_doc.toxml()}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )



    # # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )



    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
     # entity name
    entity_name = 'mobo_bot'
    # initial spawn position
    x_pos = 0; y_pos = 0; z_pos = 0
    #initial spawn orientation
    roll = 0; pitch = 0; yaw = 0
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', entity_name,
            '-x', str(x_pos), '-y', str(y_pos), '-z', str(z_pos),
            '-R', str(roll), '-P', str(pitch), '-Y', str(yaw)
            ],
        output='screen')





    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use sim time if true'),
        
        # DeclareLaunchArgument(
        #     'world',
        #     default_value=world_path,
        #     description='SDF world file',
        # ),
        

        node_robot_state_publisher,
        gazebo,
        spawn_entity,
    ])