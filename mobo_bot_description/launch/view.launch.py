import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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


    rviz_config_file_path = os.path.join(pkg_path,'config','mobo_bot_config.rviz')
    rviz2_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(rviz_config_file_path)]]
        )



    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use sim time if true'),

        node_robot_state_publisher,
        node_joint_state_publisher_gui ,
        rviz2_node,
    ])
