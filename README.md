## mobo_bot
This is an educational differential drive mobile robot created by Samuel Obiagba (samuko-things) for learning mobile robotics in ROS2. It uses the [Easy PID Motor Controller (epmc)](https://github.com/samuko-things-company/epmc_documentation) and the [Easu IMU (eimu)](https://github.com/samuko-things-company/eimu_documentation) for its base control. It also has a simulation in Gazebo Classic.

#

### mobo_bot_base
This is the base control package for the actual physical **mobo_bot** robot
the **mobo_bot_base** is meant to run on the Raspberry Pi 4 on the robot.

https://github.com/user-attachments/assets/95a72d3f-a12f-41d0-8aeb-ddb9d6ec3bd2

#

### mobo_bot_rviz
The visualization repo - **mobo_bot_rviz** - will be run on the Dev PC, which is wirelessly connected to the Pi, for visualizing the robot in RVIZ.
this package can also be used to visualize the **mobo_bot** simulation. 

#

### mobo_bot_sim
This is the mobo_bot ROS2 simulation with Gazebo Classic 11. Simulate on your PC and view the robot data in RVIZ with the **mobo_bot_rviz** package.

https://github.com/user-attachments/assets/27034f8b-ea77-4454-8616-afe079aae35d

#

### mobo_bot_nav2d
This package contains 2D navigation code implementation (based on the Nav2 ros package) for the **mobo_bot**. Learn concepts like SLAM mapping, Localization with AMCL, 2D Navigation, etc.
> [!IMPORTANT]  
> The **mobo_bot_nav2d** can be used on both the simulation - *mobo_bot_sim* - and the actual robot *mobo_bot_base*

</br></br></br>

## LETS GET STARTED
- To Simulate the **mobo_bot** on you **dev-PC** follow this [mobo_bot_sim tutorial](https://github.com/samuko-things-company/mobo_bot/blob/humble/MOBO_BOT_SIM_README.md)

- to run the actual physical **mobo_bot** robot (which uses the Raspberry Pi 4), follow this [mobo_bot_base tutorial](https://github.com/samuko-things-company/mobo_bot/blob/humble/MOBO_BOT_BASE_README.md)
