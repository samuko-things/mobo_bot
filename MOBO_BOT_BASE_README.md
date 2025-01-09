## MANUAL MOBO_BOT_BASE CONTROL SETUP (ON THE RASPBERRY PI 4B)
> [!NOTE]
> mobo_bot uses **RaspberryPi 4B** microcomputer running **Ubuntu 22.04** and **ros-humble-base**.
> You can follow this [tutorial]() to install **ros-humble-base** on **RaspberryPi 4B**

#

### Create ROS Workspace And Download and Setup Neccessary Packages

- install the `libserial-dev` package on the Raspberry Pi 4b machine
  ```shell
  sudo apt-get update
  sudo apt install libserial-dev
  ```

- install `rosdep` so you can install necessary ros related dependencies for the package.
  ```shell
  sudo apt-get update
  sudo apt install python3-rosdep2
  sudo rosdep init
  rosdep update
  ```

#

- create your <ros_ws> in the home dir. (replace <ros_ws> with your workspace name)
  ```shell
  mkdir -p ~/<ros_ws>/src
  cd ~/<ros_ws>
  colcon build
  source ~/<ros_ws>/install/setup.bash
  ```

- cd into the src folder of your <ros_ws> and download the mobo_bot packages
  ```shell
  cd ~/<ros_ws>/src
  git clone https://github.com/samuko-things-company/mobo_bot.git
  ```

- cd into the mobo_bot/mobo_bot_sim folder and add a `COLCON_IGNORE` file to the mobo_bot_sim package to prevent runnig simulation on the Raspberry Pi. *(**NOTE**: You can as well delete the mobo_bot_sim package if it pleases you)*
  ```shell
  cd ~/<ros_ws>/src/mobo_bot/mobo_bot_sim
  touch COLCON_IGNORE
  ```

- cd into the mobo_bot/mobo_bot_rviz folder and add a `COLCON_IGNORE` file to the mobo_bot_rviz package to prevent running rviz visualization on the Raspberry Pi. *(**NOTE**: You can as well delete the mobo_bot_rviz package if it pleases you)*
  ```shell
  cd ~/<ros_ws>/src/mobo_bot/mobo_bot_rviz
  touch COLCON_IGNORE
  ```
#

- go back to the `src` folder of your <ros_ws> and download and setup the `epmc_ros2_control` package for the `L298N EPMC MODULE`
  ```shell
  cd ~/<ros_ws>/src
  git clone https://github.com/samuko-things-company/epmc_ros2_control.git
  ```

- cd into the epmc_ros2_control folder and add a `COLCON_IGNORE` file to the `epmc_demo_bot_bringup` package and the `epmc_demo_bot_description` to prevent running the epmc demo bot on the Raspberry Pi. *(**NOTE**: You can as well delete them mobo_bot_sim package if it pleases you)*
  ```shell
  cd ~/<ros_ws>/src/epmc_ros2_control/epmc_demo_bot_description
  touch COLCON_IGNORE
  cd ~/<ros_ws>/src/epmc_ros2_control/epmc_demo_bot_bringup
  touch COLCON_IGNORE
  ```
  OR

  ```shell
  cd ~/<ros_ws>/src/epmc_ros2_control
  rm -rf epmc_demo_bot_description epmc_demo_bot_bringup
  ```

#

- go back to the `src` folder of your <ros_ws> and download and setup the `eimu_ros2` package for the `MPU9250 EIMU MODULE`
  ```shell
  cd ~/<ros_ws>/src
  git clone https://github.com/samuko-things-company/eimu_ros2.git
  ```

#

- install rplidar_ros binary package
  ```shell
  sudo apt install ros-humble-rplidar-ros
  ```
- go back to the `src` folder of your <ros_ws> and download rplidar_test launch file from my repo (just for testing the rplidar_A1)
  ```shell
  cd ~/<ros_ws>/src
  git clone https://github.com/samuko-things-company/rplidar_test.git
  ```
#

- cd into the root directory of your <ros_ws> and run rosdep to install all necessary ros  package dependencies
  ```shell
  cd ~/<ros_ws>/
  rosdep install --from-paths src --ignore-src -r -y
  ```
#

### Check EPMC (L298N EPMC MODULE), EIMU (MPU9250 EIMU MODULE), and RPLIDAR_A1 PORT

- check the serial port the connected sensors and motor controller
  > The best way to select the right serial port (if you are using multiple serial device) is to select by path
  ```shell
  ls /dev/serial/by-path
  ```
  > you should see a <value> (if the module is connected and seen by the computer), your serial port would be -> /dev/serial/by-path/<value>. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  > for the EPMC (i.e **L298N EPMC MODULE**), go to the mobo_bot/mobo_bot_description/urdf/**`epmc_ros2_control.xacro`** file and change the `port` parameter to the port value gotten

  > for the EIMU (i.e **MPU9250 EIMU MODULE**), go to the mobo_bot/mobo_bot_base/**`eimu_ros2_start_params.yaml`** file and change the `serial_port` parameter to the port value gotten. you can also change the `publish_frequency` to maybe 20Hz

  > for the Lidar, go to the mobo_bot/mobo_bot_base/launch/**`robot.launch.py`** file and change the `serial_port` parameter for the lidar Node to the port value gotten.

- build your <ros_ws>
  ```shell
  cd ~/<ros_ws>/
  colcon build --symlink-install
  ```

- don't forget to source your <ros_ws> in any new terminal
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ```

> [!NOTE]
> You can further edit the parameters of the yaml iles in the mobo_bot_base packge config folders


#

### Clone and Build The mobo_bot on your dev-PC connected to the raspberry PI on the mobo_bot robot

- pls follow the mobo_bot [simulation setup tutorial]() for dev-PC
- as you'll basiclly be using the mobo_bot_rviz package on your dev-PC to vizualize the robot, you can add a **COLCON_IGNORE** file to the following packages - **mobo_bot_sim**, **mobo_bot_base**, **mobo_bot_nav2d** and **mobo_bot_description**



### Run the mobo_bot_base

- on the Raspberry Pi 4b, open a new terminal and start the mobo_bot_base
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 launch mobo_bot_base robot.launch.py
  ```
> [!NOTE]
> you should see the epmc_ros2_control load and activated.
>
> if an error occoured, just ensure you are connected to the right port and run it again
>
> to run the with EIMU and sensor fusion (ekf), go to the robot.launch.py file and change the `use_ekf` launch configuration variable default value to `True`.
>
> to run the Lidar, go to the robot.launch.py file and change the `use_lidar` launch configuration variable default value to `True`.
>
> the run the above ros2 launch command again.

- on Your dev-PC, open a new terminal and start the mobo_bot_rviz by running
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 launch mobo_bot_rviz robot.launch.py
  ```
> you should now see the robot visuals on your dev-PC