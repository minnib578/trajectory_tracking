# trajectory_tracking

## How to use
Requirement: Ubuntu 18.04, ros-melodic

* create a catkin_workspace
```c
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
source devel/setup.bash
```

* download requirement dependencies and source code:turtlebot3,turtlebot3_msgs,turtlebot3_simulation (make sure download melodic version)
```c
cd src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/minnib578/Turtlebot3-trajectory-tracking.git
cd ..
catkin_make 
source devel/setup.bash
```



## Run turtlebot3
* Straghtline
```c
roslaunch trajectory_tracking gazebo_tb3_sim.launch
rosluanch trajectory_tracking stragthline.launch
```

* Circle
```c
roslaunch trajectory_tracking gazebo_tb3_sim.launch
rosluanch trajectory_tracking circle.launch
```
