### mobo_bot
This is an educational differential drive mobile robot created by Samuel Obiagba (samuko-things) for learning mobile robotics in ROS2 and micro-ros. It uses the [SMC l298N pid motor driver](http://samukothings.com/smc-l298n-pid-motor-driver-system-user-documentation/) and the [SIC MPU9250 IMU](http://samukothings.com/sic-mpu9250-imu-system-user-documentation/) for its base control. It also has a simulation in Gazebo Classic.

#

### mobo_bot_base
This is the ros2 base control package for the actual physical **mobo_bot** robot which uses the [SMC l298N pid motor driver](http://samukothings.com/smc-l298n-pid-motor-driver-system-user-documentation/) and the [SIC MPU9250 IMU](http://samukothings.com/sic-mpu9250-imu-system-user-documentation/) for its base control.
the **mobo_bot_base** is meant to run on the Raspberry Pi 4 on the robot.
> [!IMPORTANT]  
> The base repo has not been added yet but below are examples of actual robots' mobile bases I built and tested in ROS2 using the *SMC L298N pid motor driver* and *SIC MPU9250 IMU*  


https://github.com/user-attachments/assets/ad9a39bb-ade4-4895-9782-e1edabf8cb68

https://github.com/user-attachments/assets/95a72d3f-a12f-41d0-8aeb-ddb9d6ec3bd2

https://github.com/user-attachments/assets/480a1091-532a-4e97-825a-70181866c8db

#

### mobo_bot_rviz
Another Visualization repo - [**mobo_bot_rviz**](https://github.com/samuko-things-company/mobo_bot_rviz) - will be run on the Dev PC, that is wirelessly connected to the Pi, for visualizing the robot in RVIZ.
this package can also be used to visualize the **mobo_bot** simulation. 

#

### mobo_bot_sim
This is the mobo_bot ROS2 simulation with Gazebo Classic 11. here's a link to the repo: [**mobo_bot_sim**](https://github.com/samuko-things-company/mobo_bot_sim).
Simulate on your PC and view to robot data in RVIZ with the [**mobo_bot_rviz**](https://github.com/samuko-things-company/mobo_bot_rviz) package.

https://github.com/user-attachments/assets/27034f8b-ea77-4454-8616-afe079aae35d

#

### mobo_bot_nav2d
This package contains 2D navigation code implementation (based on the Nav2 ros package) for the **mobo_bot**. Learn concepts like SLAM mapping, Localization with AMCL, 2D Navigation, etc. link to the repo: [**mobo_bot_nav2d**](https://github.com/samuko-things-company/mobo_bot_nav2d)
> [!IMPORTANT]  
> The **mobo_bot_nav2d** can be used on both the simulation - *mobo_bot_sim* - and the actual robot *mobo_bot_base*
