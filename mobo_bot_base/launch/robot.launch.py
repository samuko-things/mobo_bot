import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # delare any path variable
    description_pkg_path = get_package_share_directory('mobo_bot_description')
    base_pkg_path = get_package_share_directory('mobo_bot_base')

    # Launch configuration variables specific to simulation
    use_ekf = LaunchConfiguration('use_ekf')
    use_eimu = LaunchConfiguration('use_eimu')
    test_eimu = LaunchConfiguration('test_eimu')
    use_epmc = LaunchConfiguration('use_epmc')
    odom_topic = LaunchConfiguration('odom_topic')
    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')


    declare_use_epmc_cmd = DeclareLaunchArgument(
      name='use_epmc',
      default_value='True',
      description='start EPMC with base control')
    
    declare_use_eimu_cmd = DeclareLaunchArgument(
      name='use_eimu',
      default_value='True',
      description='start IMU')
    
    declare_test_eimu_cmd = DeclareLaunchArgument(
      name='test_eimu',
      default_value='False',
      description='test IMU by publishing on map frame')
    
    declare_use_ekf_cmd = DeclareLaunchArgument(
      name='use_ekf',
      default_value='True',
      description='fuse odometry and imu data if true')

    declare_odom_topic_cmd = DeclareLaunchArgument(
      name='odom_topic',
      default_value='odom',
      description='topic to remap /odometry/filtered to')
    
    declare_lidar_cmd = DeclareLaunchArgument(
      name='use_lidar',
      default_value='True',
      description='use rplidar A1 if true')
    
    declare_camera_cmd = DeclareLaunchArgument(
      name='use_camera',
      default_value='True',
      description='use camera if true')
    
    # create needed nodes or launch files
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(description_pkg_path,'launch','rsp.launch.py')]), 
        launch_arguments={'use_sim_time': 'False',
                          'use_simulation': 'False'}.items(),
        condition=IfCondition(use_epmc))
    

    robot_controllers = os.path.join(base_pkg_path,'config','epmc_diff_drive_controller.yaml')

    # see -> https://github.com/ros-controls/ros2_control_demos/blob/humble/example_2/bringup/launch/diffbot.launch.py
    # see -> https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html
    controller_manager_with_ekf = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        condition=IfCondition(PythonExpression([use_epmc, ' and ', use_ekf])),
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/epmc_diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/epmc_diff_drive_controller/odom", "/wheel/odometry"),
        ],
    )

    controller_manager_without_ekf = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        condition=IfCondition(PythonExpression([use_epmc, ' and not ', use_ekf])),
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/epmc_diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/epmc_diff_drive_controller/odom", odom_topic),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        condition=IfCondition(use_epmc)
    )

    epmc_diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["epmc_diff_drive_controller"],
        condition=IfCondition(use_epmc)
    )    

    # Delay start of robot_controller after `joint_state_broadcaster`
    start_epmc_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[epmc_diff_drive_controller_spawner],
        )
    )


    eimu_ros_config_file = os.path.join(base_pkg_path,'config','eimu_ros_start_params.yaml')

    eimu_ros_node = Node(
        package='eimu_ros',
        executable='eimu_ros',
        name='eimu_ros',
        output='screen',
        parameters=[
            eimu_ros_config_file,
            {'publish_tf_on_map_frame': test_eimu}
        ],
        condition=IfCondition(use_eimu)
    )

    ekf_config_path = os.path.join(base_pkg_path,'config','ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path
        ],
        condition=IfCondition(PythonExpression([use_eimu, ' and ', use_ekf])),
        remappings=[("odometry/filtered", odom_topic)]
    )
    
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{'channel_type': 'serial',
                      'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2.2:1.0-port0',
                      'serial_baudrate': 115200,
                      'frame_id': 'lidar',
                      'inverted': False,
                      'angle_compensate': True,
                      'scan_mode': 'Sensitivity'}
                      ],
        condition=IfCondition(use_lidar),
        remappings=[("/scan", "/lidar/scan")],
        output='screen'
    )

    camera_node = Node(
        package='opencv_ros_camera',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{'frame_id': "camera_optical",
                      'port_no': 0,
                      'frame_width': 320,
                      'frame_height': 240,
                      'compression_format': "jpeg", # you can also use "jpeg"
                      'publish_frequency': 30.0}
                    ],
        condition=IfCondition(use_camera),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # add the necessary declared launch arguments to the launch description
    ld.add_action(declare_use_ekf_cmd)
    ld.add_action(declare_use_eimu_cmd)
    ld.add_action(declare_test_eimu_cmd)
    ld.add_action(declare_use_epmc_cmd)
    ld.add_action(declare_odom_topic_cmd)
    ld.add_action(declare_lidar_cmd)
    ld.add_action(declare_camera_cmd)
    

    # Add the nodes to the launch description
    ld.add_action(rsp_launch)
    ld.add_action(controller_manager_with_ekf)
    ld.add_action(controller_manager_without_ekf)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(start_epmc_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner)
    ld.add_action(eimu_ros_node)
    ld.add_action(ekf_node)
    ld.add_action(lidar_node)
    ld.add_action(camera_node)

    return ld      # return (i.e send) the launch description for excecution
