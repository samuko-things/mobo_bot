## SETUP MOBO_BOT_SIM ON YOUR DEV PC
> [!NOTE]
> Your Dev PC must be running **Ubuntu 22.04** and **ros-humble-desktop** with **gazebo igintion fortress**.
> You can follow this [tutorial]() to install **ros-humble-desktop** on **PC**

#

### Create ROS Workspace And Download and Setup mobo_bot Packages
- install python pynput library for the mobo_bot teleop
  ```shell
  pip3 install pynput
  ```

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

- cd into the mobo_bot/mobo_bot_base folder and add a `COLCON_IGNORE` file to the mobo_bot_base package to prevent the running of mobo_bot_base . *(**NOTE**: You can as well delete the mobo_bot_base package if it pleases you)*
  ```shell
  cd ~/<ros_ws>/src/mobo_bot/mobo_bot_base
  touch COLCON_IGNORE
  ```

- cd into the root directory of your <ros_ws> and run rosdep to install all necessary ros  package dependencies
  ```shell
  cd ~/<ros_ws>/
  rosdep install --from-paths src --ignore-src -r -y
  ```

- build your <ros_ws>
  ```shell
  cd ~/<ros_ws>/
  colcon build --symlink-install
  ```

- don't forget to source your <ros_ws> in any new terminal
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ```

#

### Run the mobo_bot_sim

- on your dev-PC, open a new terminal and start the mobo_bot_sim
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 launch mobo_bot_sim sim.launch.py
  ```
- in a differnt terminal, run the mobo_bot_teleop to drive the robot around using the arrow keys on your keyboard
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 run mobo_bot_teleop mobo_bot_teleop
  ```
  OR
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 run mobo_bot_teleop mobo_bot_teleop <v in m/s> <w in rad/sec>
  ```

- to tryout the 2D navigation with simulation (using the existing world and map):
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 launch mobo_bot_sim nav_bringup.launch.py
  ```

- to tryout the 2D navigation with simulation with slam:
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 launch mobo_bot_sim slam_nav_bringup.launch.py
  ```

- to tryout only the amcl localization so see how it works:
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 launch mobo_bot_sim amcl.launch.py
  ```

- to build map of the world with slam run:
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 launch mobo_bot_sim slam_mapping.launch.py
  ```


## Drive your robot with teleop



NOTE that the package publishes to the /cmd_vel topic. It requires you to set the linear velocity (v) and angular velocity (w), you want your robot to move at, as argument to run the package. Below is an example of how to run the package to drive your robot.

  ```shell
  ros2 run arrow_key_teleop_drive arrow_key_teleop_drive 0.2 1.0
  ```
drive the robot easily using the arrow keys