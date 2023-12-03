# COSC 081/281 - Principles of Robot Design and Programming (Term: Spring, Year: 2023)

## Assignment: PA3 - README / Instructions
## Author: Carter Kruse
## Date: May 3, 2023

---

### Install Docker

[Installation Instructions for Mac](https://docs.docker.com/docker-for-mac/install/)

### ROS (Apple Silicon)

1. Open a terminal and clone the following repository with the command `git clone https://github.com/jaismith/ros-apple-silicon`.
2. Enter into the cloned repository folder, `cd ros-apple-silicon`.
3. Run `docker-compose up --build`.

### Set Up (Roscore, Map, Transform)

Once the other terminal shows the various completion messages and remains running without errors, open another terminal.

1. Enter into the cloned repository folder, `cd ros-apple-silicon`.
2. Run `docker-compose exec ros bash` (`docker-compose up` has to be running).
3. Run `roscore`.

Open another terminal.

1. Enter into the cloned repository folder, `cd ros-apple-silicon`.
2. Run `docker-compose exec ros bash` (`docker-compose up` has to be running).
3. Run `rosrun map_server map_server ~/catkin_ws/PA3/maze.yml` to load the map.

If this was unsuccessful (i.e. the 'map_server' is not installed), run the following command:
`sudo apt install ros-melodic-map-server`

Open another terminal.

1. Enter into the cloned repository folder, `cd ros-apple-silicon`.
2. Run `docker-compose exec ros bash` (`docker-compose up` has to be running).
3. Run `rosrun tf static_transform_publisher x y 0 0 0 0 /map /odom 50` to specify the transform from the 'map' to the 'odom' reference frame.

### Running Simulation

Open another terminal.

1. Enter into the cloned repository folder, `cd ros-apple-silicon`.
2. Run `docker-compose exec ros bash` (`docker-compose up` has to be running).
3. Run `rviz` to start the visualization. Click 'Panels->Add New Panel->Tool Properties'.
4. Add the 'Map' and 'PoseArray' elements with '/map' and '/pose_sequence' topics respectively to display the visualization.

Open another terminal.

1. Enter into the cloned repository folder, `cd ros-apple-silicon`.
2. Run `docker-compose exec ros bash` (`docker-compose up` has to be running).
3. Enter into the relevant folder, `cd PA3`.
4. Run `rosrun stage_ros stageros maze.world` to start the simulation.

Open another terminal.

1. Enter into the cloned repository folder, `cd ros-apple-silicon`.
2. Run `docker-compose exec ros bash` (`docker-compose up` has to be running).
3. Enter into the relevant folder, `cd PA3`.
4. Run `python PA3_Planning.py`.

To see whether it was successful, open your browser to `localhost:8080/vnc.html` and click connect. The robotic simulator is now running in your browser.

To see the trail of the robot, enable it from the simulator GUI: 'View->Trails->Fast' (CTRL + "F")

### Termination

To end the operation of any terminal press CTRL + C, which will stop the execution. Once a terminal is stopped -- you should see it as the terminal that can accept commands -- press CTRL + D to exit. Do so with the simulator prior to stopping the virtual environment, prior to stopping the roscore, prior to stopping the Docker container.

When the main Docker container is terminated, you should see completion messages. At this point, all terminals can be closed if you wish.
