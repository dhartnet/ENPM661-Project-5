# ENPM-661-Project5
Project 5 of ENPM-661 Planning For Autonomous Robots Course
# Written by: Rey Roque-Pérez and Dustin Hartnett
# UID: reyroque: 120498626
# UID: dhartnet: 115262249
Project 5 of ENPM-661 Planning For Autonomous Robots Course

Implementation of the Lazy PRM Algorithm for a Mobile with ROS/Gazebo Simulation

# How to Run
Download the ros package to a location in your computer

In the terminal navigate to where the ros package is saved and then navigate to <project3_ws/src/turtlebot3_project3/scripts>

Build your package and launch "ros2 launch turtlebot3_project3 competition_world.launch.py"

Run "python3 PRM_ros.py" in a new terminal, but ensure you are also in the <project3_ws/src/turtlebot3_project3/scripts> location
  -This python script runs the script "LazyPRM.py" and calls upon its outputs to use with ROS

The start and goal coordinates are currently fixed to maximise the distance and complexity the algorithm has to search for a path. Gazebo struggled with this long distance, though. 

To change the goal coordinates, edit the "LazyPRM.py" file. At the bottom of the code you will see a heavily commented section where the goal coordinate can be changed. For short trips that seemed to work best in Gazebo, we recommend (200, 125, 0), or something similar.

# Dependencies
The following Python libraries are used:

opencv-python: For visualizing the workspace and creating a video.

numpy: For resizing the arrays used to draw the obstacles with opencv.

random: For randomizing nodes

heapq: For the heap function

time: For calculating the duration of computation

geometry_msgs.msg: For gazebo implementation

rclpy: For gazebo implementation

# Output
A video file called output.mp4 will be generated in the same directory were the code is ran from after it is done running.

A video file showing our gazebo implementation is also in the Git repository

The following link also contains both videos in a Google Drive: https://docs.google.com/document/d/1yUDu1JPfzse9m4RtQzrcgh_KQsngioZwCB8ETkK2R-Y/edit?usp=sharing

# Github Repository: https://github.com/dhartnet/ENPM661-Project-5
 
# ENPM661-Project-5
