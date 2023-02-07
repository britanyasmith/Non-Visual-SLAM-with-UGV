# UGV LiDAR Team
<p align="center">
  <img src=https://media.giphy.com/media/fIqglx81Ch5N0L8BqU/giphy-downsized-large.gif />
</p>

## This branch contains UGVLidar instructions for performing autonomous navigation and obstacle avoidance using the Husky Robot for sp22Robot. This includes the instructions for performing all these tasks within Ubuntu and ROS as well as the code to manually move the robot a certain x, y, and theta distance. 
## Hardware Requirements for Navigation and Obstacle Avoidance: 
- 2D RPLidar
- Nvidia Jetson Nano
- Husky Clearpath Robot https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/
## Software Requirements for perception
- RViz
- ROS Melodic [Run this command line: sudo apt install ros-melodic-Husky]
- Ubuntu Bionic 18.04

## Approach
### 
Our focus is to generate a map of the space, perform navigation and obstacle avoidance to estimated location, and finally communicate with the ARM team via MQTT to move robot to the target location.

<p align="center">
  <img src="/pipeline.png" />
</p>
<p align="center">
  <img width="500" height="400" src="/path.png" />
</p>

## STEPS TO CREATING A MAP OF THE SPACE 

### Step 1: When rplidar_ros.launch file does not exist nor the name of it is not rplidar_ros
1. ls -l /dev/ttyUSB0
2. sudo chmod 666 /dev/ttyUSB0
3. cd /home/doyun/catkin_ws
4. catkin_make rplidarNode
5. source ~/catkin_ws/devel/setup.bash
6. roslaunch rplidar_ros rplidar.launch
7. Lidar should be spinning
8. Leave this terminal open

### Step 2: Husky visualization (https://www.clearpathrobotics.com/assets/guides/noetic/husky/InterfacingWithHusky.html)
1. Open New terminal 
2. roslaunch husky_viz view_robot.launch
3. Leave this terminal open

### Step 3: Run Gmapping
1. Open a new terminal
2. roslaunch husky_navigation gmapping_demo.launch
3. Open Rviz
4. Under Sensing then Laser Scan, set Decay time to 10
5. Check navigation. Unselect Global and Local Cost maps
6. Uncheck Odometry

### Step 4: Connect Controller 
1. Systems Settings -> Bluetooth -> Plus button
2. Turn on controller by clicking PS4 controller button and share button at same time until you see blinking light 
3. Search in devices until you see "wireless controller"
4. Select and click "Next" and the light will go solid 
5. Drive robot around to create map (watch the terminal to ensure when a loop is skipped, stop the husky to allow for recordination)

### Step 5: Save Map (DO NOT MOVE HUSKY)
1. Open new terminal 
2. rosrun map_server map_saver -f rm1311_032222
3. Close gmapping terminal 

## STEPS TO AUTONOMOUS NAVIGATION 

### Step 1: When rplidar_ros.launch file does not exist nor the name of it is not rplidar_ros
1. ls -l /dev/ttyUSB0
2. sudo chmod 666 /dev/ttyUSB0
3. cd /home/doyun/catkin_ws
4. catkin_make rplidarNode
5. source ~/catkin_ws/devel/setup.bash
6. roslaunch rplidar_ros rplidar.launch
7. Lidar should be spinning
8. Leave this terminal open

### Step 2: Connect Controller 
1. Systems Settings -> Bluetooth -> Plus button
2. Turn on controller by clicking PS4 controller button and share button at same time until you see blinking light 
3. Search in devices until you see "wireless controller"
4. Select and click "Next" and the light will go solid 
5. Drive robot around to create map (watch the terminal to ensure when a loop is skipped, stop the husky to allow for recordination)

### Step 3: Husky visualization
1. Open New terminal 
2. roslaunch husky_viz view_robot.launch config:=localization
3. Leave this terminal open

### Step 4: Upload 2D Pregenerated Map of space
1. Open new terminal 
2. roslaunch husky_navigation amcl_demo.launch map_file:=/home/doyun/rm1311_042122.yaml
3. Under Global Options, choose "map" for fixed frame 
4. In RVIZ window check navigation box
5. Under Navigation -> Exploration Costmap, choose  "/map" for the topic 
6. Uncheck Odometry

### Step 5: Orient Robot 
1. Use 2D Pose estimate to line up the robot with its current position 
2. Try to aline the laser readings (denoted by the yellow line) with the pregenerated map edges 

### Step 6: Navigation
1. Use 2D Nav Goal to send the desired trajectory, distance, and final orientation for the robot 


## STEPS TO RUNNING THE INTEGRATION 

## Step 1: restart ros
1. sudo systemctl restart ros

## Step 2: When the simple_controller cannot be found 
1. cd /home/doyun/catkin_ws
2. catkin_make simple_controller
3. source ~/catkin_ws/devel/setup.bash
4. rosrun simple_controller Pos&Ori_Adjustment.py
