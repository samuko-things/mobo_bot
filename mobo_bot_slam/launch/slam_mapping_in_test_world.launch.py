# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os


from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import yaml
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess

import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory('mobo_bot_description')

    xacro_file = os.path.join(pkg_path,'urdf','robot_urdf_description.xacro')
    robot_description_doc = xacro.parse(open(xacro_file))
    xacro.process_doc(robot_description_doc)

    world_file_name = 'test_world.world'
    world_path = os.path.join(pkg_path, 'world', world_file_name)



    params = {'robot_description': robot_description_doc.toxml()}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    rviz_config_file_path = os.path.join(get_package_share_directory('mobo_bot_slam'),'config','mobo_bot_slam_config.rviz')
    rviz2_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(rviz_config_file_path)]]
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


    slam_mapping_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("mobo_bot_slam"), 'config', 'slam_mapping_params.yaml')],
        #     remappings=[
        #     # ('/odom', '/turtlesim1/turtle1/pose'),
        #     # ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        # ]
        )


    return LaunchDescription([
        DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'),
        
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='SDF world file',
        ),

        # ExecuteProcess(cmd=['gazebo', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        ExecuteProcess(cmd=['gazebo', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

        node_robot_state_publisher,
        rviz2_node,
        spawn_entity,
        slam_mapping_node,
        
    ])