## Working with the Physical MoboBot
> [!NOTE]
> **MoboBot** uses **RaspberryPi 4B** microcomputer running **Ubuntu 22.04** and **ros-jazzy-base**.

![mobo_bot_amcl](./docs//mobo_bot_amcl.gif)

#

### Prerequisite Dependencies

- install the `libserial-dev` package on the Raspberry Pi 4b machine
  ```shell
  sudo apt-get update
  sudo apt install libserial-dev
  ```
- install cyclone DDS (if you have not) on the Raspberry Pi 4b machine
  ```shell
  sudo apt install ros-jazzy-rmw-cyclonedds-cpp
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
  ```
  
#

### Create ROS Workspace And Download the MoboBot packages

- create your mobo_bot_ws in the home dir.
  ```shell
  mkdir -p ~/mobo_bot_ws/src
  cd ~/mobo_bot_ws
  colcon build
  source ~/mobo_bot_ws/install/setup.bash
  ```

- cd into the src folder of your mobo_bot_ws and download the mobo_bot packages
  ```shell
  cd ~/mobo_bot_ws/src
  git clone -b jazzy https://github.com/robocre8/mobo_bot.git
  ```

- cd into the mobo_bot/mobo_bot_sim folder and add a `COLCON_IGNORE` file to the mobo_bot_sim package to prevent runnig simulation on the Raspberry Pi. 
  ```shell
  cd ~/mobo_bot_ws/src/mobo_bot/mobo_bot_sim
  touch COLCON_IGNORE
  ```

- cd into the mobo_bot/mobo_bot_rviz folder and add a `COLCON_IGNORE` file to the mobo_bot_rviz package to prevent running rviz visualization on the Raspberry Pi.
  ```shell
  cd ~/mobo_bot_ws/src/mobo_bot/mobo_bot_rviz
  touch COLCON_IGNORE
  ```
  
#

### Download the ROS2 packages and Drivers for the Sensor and Actuators Used by MoboBot

- create a folder called **hardware**
  ```shell
  cd ~/mobo_bot_ws/src
  mkdir hardware
  ```

#### EPMC Motor Driver
- go back to the `src` folder of your mobo_bot_ws and download and setup the `epmc_ros_hw_plugin` ros2 package
  ```shell
  cd ~/mobo_bot_ws/src/hardware
  git clone https://github.com/robocre8/epmc_ros_hw_plugin.git
  ```

#### EIMU Module
- go back to the `src` folder of your mobo_bot_ws and download and setup the `eimu_ros` ros2 package
  ```shell
  cd ~/mobo_bot_ws/src/hardware
  git clone https://github.com/robocre8/eimu_ros.git
  ```

#### RPLIDAR C1
- go back to the `src` folder of your mobo_bot_ws and download sllidar ros2 for RPLIDAR C1
  ```shell
  cd ~/mobo_bot_ws/src/hardware
  git clone https://github.com/Slamtec/sllidar_ros2.git
  ```

#### CAMERA (with OpenCV)
- install opencv on the Raspberry Pi 4b machine
  ```shell
  sudo apt install libopencv-dev python3-opencv
  pip3 install opencv-python
  ```

- go back to the `src` folder of your mobo_bot_ws and download the opencv_ros_camera package, from robocre8, for working with the USB camera
  ```shell
  cd ~/mobo_bot_ws/src/hardware
  git clone https://github.com/robocre8/opencv_ros_camera.git
  ```

#

### Check EPMC (L298N EPMC MODULE), EIMU (MPU9250 EIMU MODULE), RPLIDAR_A1 AND USB Camera PORTS

- cd into the root directory of your mobo_bot_ws and run rosdep to install all necessary ros  package dependencies
  ```shell
  cd ~/mobo_bot_ws/
  rosdep install --from-paths src --ignore-src -r -y
  ```
  
#

### Check EPMC (L298N EPMC MODULE), EIMU (MPU9250 EIMU MODULE), RPLIDAR_A1 AND USB Camera PORTS

- check the serial port the connected sensors and motor controller
  > The best way to select the right serial port (if you are using multiple serial device) is to select by path
  ```shell
  ls /dev/serial/by-path
  ```
  > you should see a <value> (if the module is connected and seen by the computer), your serial port would be -> /dev/serial/by-path/<value>. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  > **for the EPMC** (i.e **L298N EPMC MODULE**), go to the `mobo_bot/mobo_bot_description/urdf/`**`epmc_ros2_control.xacro`** file and change the `port` parameter to the port value gotten

  > **for the EIMU** (i.e **MPU9250 EIMU MODULE**), go to the `mobo_bot/mobo_bot_base/`**`eimu_ros_start_params.yaml`** file and change the `port` parameter to the port value gotten. you can also change the `publish_frequency` to maybe 20Hz

  > **for the Lidar**, go to the `mobo_bot/mobo_bot_base/launch/`**`robot.launch.py`** file and change the `serial_port` parameter for the lidar Node to the port value gotten.
  > ````
  > lidar_node = Node(
  >     package='rplidar_ros',
  >     executable='rplidar_node',
  >     name='rplidar_node',
  >     parameters=[{'channel_type': 'serial',
  >                   'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1.2:1.0-port0',
  >                   'serial_baudrate': 115200,
  >                   'frame_id': 'lidar',
  >                   'inverted': False,
  >                   'angle_compensate': True,
  >                   'scan_mode': 'Sensitivity'}
  >                   ],
  >     condition=IfCondition(use_lidar),
  >     remappings=[("/scan", "/lidar/scan")],
  >     output='screen'
  > )
  > ````

  > **for the USB Camera**, the video port no, which is by default 0, and every other should be okay. <br/>
  > But if you still intend to adjust anything, go to the `mobo_bot/mobo_bot_base/launch/`**`robot.launch.py`** file to change any of the camera parameters
  > 
  > ````
  > camera_node = Node(
  >     package='opencv_ros_camera',
  >     executable='camera_publisher',
  >     name='camera_publisher',
  >     output='screen',
  >     parameters=[{'frame_id': "camera_optical",
  >                   'port_no': 0,
  >                   'frame_width': 320,
  >                   'frame_height': 240,
  >                   'compression_format': "jpeg", # you can also use "jpeg"
  >                   'publish_frequency': 30.0}
  >                 ],
  >     condition=IfCondition(use_camera),
  > )
  > ````

#

### Build The MoboBot Packages and the Different Hardware Packages

- build your mobo_bot_ws
  ```shell
  cd ~/mobo_bot_ws/
  colcon build --symlink-install
  ```

- don't forget to source your mobo_bot_ws in any new terminal
  ```shell
  source ~/mobo_bot_ws/install/setup.bash
  ```

> [!NOTE]
> You can further edit the parameters of the .yaml files in the mobo_bot_base package config folders

#

### Clone and Build The MoboBot packages on your dev-PC connected (via ssh) to the Raspberry PI on the MoboBot robot

- pls follow the [mobo_bot_sim tutorial](https://github.com/robocre8/mobo_bot/blob/jazzy/MOBO_BOT_SIM_README.md) for dev-PC
- you'll be using the mobo_bot_rviz package on your dev-PC to visualize the robot.


#

### Run the Physical MoboBot

![mobo_bot_base_drive](./docs/mobo_bot_drive_test.gif)

- on the Raspberry Pi 4b, open a new terminal and start the mobo_bot_base
  ```shell
  source ~/mobo_bot_ws/install/setup.bash
  ros2 launch mobo_bot_bringup robot.launch.py
  ```
> [!NOTE]
> If any error occurs, first unplug the lidar (from the USB HUB) and plug it back
> then unplug the USB HUB from the Raspberry Pi Port and plug it back.
> Everything should now work.
> ros2 launch mobo_bot_bringup robot.launch.py

- on Your dev-PC, open a new terminal and start the mobo_bot_rviz by running
  ```shell
  source ~/mobo_bot_ws/install/setup.bash
  ros2 launch mobo_bot_rviz robot.launch.py
  ```
> You should now see the robot visuals on your dev-PC

#

### Drive MoboBot with a special arrow-key teleop form the DevPC (not on the Pi)
- in a different terminal on the Dev PC (not on the Pi), run the arrow_key_teleop_drive to drive the robot around using the arrow keys on your keyboard
  ```shell
  source ~/mobo_bot_ws/install/setup.bash
  ros2 run arrow_key_teleop_drive arrow_key_teleop_drive
  ```
  OR
  ```shell
  source ~/mobo_bot_ws/install/setup.bash
  ros2 run arrow_key_teleop_drive arrow_key_teleop_drive <v in m/s> <w in rad/sec>
  ```