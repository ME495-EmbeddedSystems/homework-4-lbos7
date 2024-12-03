# ME495 Embedded Systems Homework 4
Author: Logan Boswell

This package uses a differential drive robot to explore and map a world in Gazebo.

The algorithm used for autonomous mapping consists of the following steps:
1. Pick location in the unexplored (grey) region of the map and set as the goal position
2. Wait until the commanded velocity is zero (either the robot has reached the goal or is unable to) and then start over with step 1

## Quickstart
1. Use `ros2 launch nubot_nav manual_explore.launch.xml` to explore the enviroment manually. In order to move the robot, you must select the "2D Goal Pose" button at the top of the rviz window and then click a desired pose in rviz
2. Here is an example of manual exploration
    <video src="https://github.com/user-attachments/assets/70314d97-1bcc-46bd-a4af-085535d3de35" width="500" />
3. Use `ros2 launch nubot_nav explore.launch.xml` to explore the enviroment without manual inputs.
4. Here is an example of autonomous exploration
    <video src="https://github.com/user-attachments/assets/68dc8fef-2e2f-409c-888b-440d96f29c53" width="500" />