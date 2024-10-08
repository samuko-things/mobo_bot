## SETUP MOBO_BOT_SIM ON YOUR DEV PC
> [!NOTE]
> Your Dev PC must be running **Ubuntu 22.04** and **ros-humble-desktop**.
> You can follow this [tutorial]() to install **ros-humble-desktop** on **PC**

#

### Create ROS Workspace And Download and Setup mobo_bot Packages

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
  OR

  ```shell
  cd ~/<ros_ws>/src/mobo_bot
  rm -rf mobo_bot_base
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

- you can also run the navigation with the simulation also also
  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 launch mobo_bot_nav2d build_map_with_slam.launch.py
  ```

  ```shell
  source ~/<ros_ws>/install/setup.bash
  ros2 launch mobo_bot_nav2d localize_with_amcl.launch.py
  ```