# Robot-Navigation-Using-A-Star-Path-Finding-Algorithm




Example Video Trial 1 



https://github.com/user-attachments/assets/54077955-902f-41e3-97b6-fe03b8679daf


# A-Star algorithm preview 
Example of A*: The shortest path from start point to end point 
<img width="640" alt="Trial 1 Figure 2" src="https://github.com/user-attachments/assets/59ded6a8-ec72-488e-b1b9-6021e02b4ebc">

# How to run 
The algorithm is tested in ubuntu system using ROS.
First, clone turtlebot3 simulator. 
```
$ mkdir catkin_ws && cd catkin_ws
$ mkdir src && cd src
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ..
```
Then, copy the files in launch, worlds, and models to /src/turtlebot3_simulations/turtlebot3_gazebo/launch, worlds, and models
```
$ catkin_make
```
To run the simulator
```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_maze.launch
```
To start A* navigation
```
$ cd beginner/
$ phthon3 run_aster_navigation.py
```
Type start point and end point in terminal




