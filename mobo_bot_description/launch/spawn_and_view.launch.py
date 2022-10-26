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

    # world_file_name = 'simple_house.world'
    # world_path = os.path.join(pkg_path, 'world', world_file_name)



    params = {'robot_description': robot_description_doc.toxml()}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )


    rviz_config_file_path = os.path.join(pkg_path,'config','mobo_bot_config.rviz')
    rviz2_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(rviz_config_file_path)]]
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
        # node_joint_state_publisher_gui ,
        # node_joint_state_publisher ,
        rviz2_node,
        gazebo,
        spawn_entity,
    ])