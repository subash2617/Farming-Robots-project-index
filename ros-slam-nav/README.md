# SLAM Navigation

### 1. Simulation
 
#### 1.1. World simulation
There are many virtual environments that availabled in robot_gazebo's launch folder. Run below command to open virtual house environment. 
```
roslaunch robot_gazebo robot_house.launch
```

In order to teleoperate the robot with the keyboard,  launch the teleoperation node with below command in a new terminal window.
```
roslaunch robot_teleop robot_teleop_key.launch
```

#### 1.2. SLAM simulation
The following instructions require prerequisites from the previous section. 
Open a new terminal window from the PC and run  the SLAM node. Hector SLAM method is used by default. 
```
roslaunch robot_slam robot_slam.launch slam_methods:=gmapping
```
When the map is created successfully, open a new terminal from remote PC and save the map.
```
rosrun map_server map_saver -f ~/map
 ``` 
#### 1.3. Navigation simulation
Just like the SLAM simulation, Navigation simulation also requires prerequisites from world simulation section. 
Open a new terminal and run the Navigation node.
```
roslaunch robot_navigation robot_navigation.launch
``` 
 
### 2. SLAM
Run roscore from remote PC. 
```
roscore 
```
 
Open a new terminal from Single Board Computer (SBC) on the robot and launch the Bringup. 
```
roslaunch robot_bringup robot.launch
```

Open a new terminal from remote PC and launch the SLAM node. 
```
roslaunch robot_slam robot_slam.launch
```
Run the teleoperation node to control the robot. 
```
roslaunch robot_teleop robot_teleop_key.launch 
```
Save the map. 
```
rosrun map_server map_saver -f ~/map
```
 
### 3. Navigation
Run roscore from remote PC. 
```
roscore 
```
 
Open a new terminal from Single Board Computer (SBC) on the robot and launch the Bringup. 
```
roslaunch robot_bringup robot.launch
```

Open a new terminal from remote PC and launch the Navigation node. 
```
roslaunch robot_navigation robot_navigation.launch
```
 
 



