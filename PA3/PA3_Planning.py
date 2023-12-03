#!/usr/bin/env python

# Author: Carter Kruse
# Date: May 3, 2023

# Import Relevant Libraries
import rospy # Module - ROS APIs
import tf
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Pose, PoseArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Import Python Modules
import math
import numpy as np
from enum import Enum
from collections import deque

# Constants
# Topic Names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_ODOM_TOPIC = 'odom'
DEFAULT_POSE_SEQ_TOPIC = 'pose_sequence'

# Velocities
LINEAR_VELOCITY = 0.1 # m/s
ANGULAR_VELOCITY = math.pi / 8 # rad/s

# Tolerances
ANGLE_TOLERANCE = 0.04
DISTANCE_TOLERANCE = 0.12

# Goal Location (Map Reference Frame)
GOAL_X = 2
GOAL_Y = 7

# ROBOT DIMENSIONS
DIMENSIONS = 0.3

# Frequency at which the loop operates.
FREQUENCY = 10 # Hz

def bfs(maze, start, goal):
    """Breadth First Search Algorithm"""
    # Define the possible moves from a cell/node (only considering adjacent cells).
    moves = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    # Create a set to keep track of the visited cells.
    visited = set()

    # Create a queue.
    queue = deque()

    # Add the starting cell to the queue, marking it as visited.
    queue.append(start)
    visited.add(start)

    # Create a dictionary to store the parent of each visited cell.
    parent = {}
    parent[start] = None

    # Loop until the queue is empty or the goal cell is found.
    while queue:
        # Pop the first cell from the queue.
        current = queue.popleft()

        # Check if the current cell is the goal cell.
        if current == goal:
            # Reconstruct the path using the 'parent' dictionary.
            path = []
            while current:
                path.append(current)
                current = parent[current]
            
            # Reverse the path and return it.
            return path[::-1]
        
        # Otherwise, check the possible moves from the current cell.
        for move in moves:
            # Determine the next cell.
            next_cell = (current[0] + move[0], current[1] + move[1])

            # Check if the next cell is within the maze boundaries and is free.
            if 0 <= next_cell[0] < len(maze) and 0 <= next_cell[1] < len(maze[0]) and maze[next_cell[1]][next_cell[0]] == 0:
                # Check if the next cell has not been visited before.
                if next_cell not in visited:
                    # Add the next cell to the queue, mark it as visited, and set its parent.
                    queue.append(next_cell)
                    visited.add(next_cell)
                    parent[next_cell] = current

    # If the goal cell was not found, return an empty path.
    return []

# # # # # # # # # #

class FSM(Enum):
    MOVE = 0
    STOP = 1

class Planning():
    """Class - ROS Node"""
    def __init__(self, linear_velocity = LINEAR_VELOCITY, angular_velocity = ANGULAR_VELOCITY, 
                 angle_tolerance = ANGLE_TOLERANCE, distance_tolerance = DISTANCE_TOLERANCE,
                 goal_x = GOAL_X, goal_y = GOAL_Y):
        """Initialization Function / Constructor"""
        # Set Up (Publishers / Subscribers)
        # Publisher - Sends velocity commands.
        self.cmd_vel_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size = 1)

        # Publisher - Sends the corresponding pose sequences.
        self.pose_pub = rospy.Publisher(DEFAULT_POSE_SEQ_TOPIC, PoseArray, queue_size = 10)

        # Subscriber - Recieves messages from the odometer.
        self.odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self.odom_callback, queue_size = 1)

        # Parameters
        self.linear_velocity = linear_velocity # Constant Linear Velocity
        self.angular_velocity = angular_velocity # Constant Angular Velocity

        # Goal Location
        self.goal_x = goal_x
        self.goal_y = goal_y

        # Current Position & Orientation
        self.x_position = 0.0
        self.y_position = 0.0
        self.orientation = 0.0

        # Angle & Position Tolerances
        self.angle_tolerance = angle_tolerance
        self.distance_tolerance = distance_tolerance

        # FSM Variable
        self.fsm = FSM.MOVE

        # Transform Listener
        self.transform = tf.TransformListener()

        # Path
        self.path = []

        # Resolution
        self.resolution = 0.05

        # Wait for a few seconds for the registration (to ROS master).
        rospy.sleep(3.0)

    def odom_callback(self, odom_msg):
        """Set the position and orienation of the robot."""
        # Get the current position and orientation of the robot.
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation

        # Set the current position.
        self.x_position = position.x
        self.y_position = position.y

        # Convert the orientation from quaternion to euler angles.
        (_, _, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Set the current orientation.
        self.orientation = yaw

        """Publish the pose sequence of the robot."""
        # Read the corresponding path (a list of (x, y) tuples) and publish display it.
        # Create a PoseArray message, and write the corresponding header.
        pose_seq_msg = PoseArray()
        pose_seq_msg.header.stamp = rospy.Time.now()

        # The 'map' reference frame is used for the PoseArray.
        pose_seq_msg.header.frame_id = "map"

        # Fill in the appropriate positions and orientations, according to the path.
        for i, (x, y) in enumerate(self.path):
            # Create a pose, with the corresponding position/orientation.
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0

            # The last pose should have the same orientation as the previous.
            if i == len(self.path) - 1:
                pose.orientation = last_orientation
            
            else:
                # Determine the orientation to the next point in the path.
                next_x, next_y = self.path[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
                quaternion = quaternion_from_euler(0, 0, yaw)

                # Add the orientation values to the pose.
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
            
            # Add the pose to the list (which is the relevant message).
            pose_seq_msg.poses.append(pose)

            # Update the last orientation.
            last_orientation = pose.orientation

        # Publish the pose sequence.
        self.pose_pub.publish(pose_seq_msg)

    def find_path(self, start, goal):
        """Determine the path of the robot, according to an occupancy grid."""
        # Wait for the OccupancyGrid message (according to the "/map" topic).
        grid_msg = rospy.wait_for_message("/map", OccupancyGrid)

        # Extract relevant information (width, height) from the message.
        width = grid_msg.info.width
        height = grid_msg.info.height
        resolution = grid_msg.info.resolution

        # Extra Information
        # origin = grid_msg.info.origin.position
        # orientation = grid_msg.info.origin.orientation

        # Read the occupancy grid and create a numpy array.
        occupancy_grid = np.array(grid_msg.data).reshape((height, width))

        # Determine the distance away from an obstacle the robot should stay.
        distance = int(DIMENSIONS / resolution)

        # Create a copy of the original occupancy grid.
        new_grid = np.copy(occupancy_grid)

        # Iterate over each cell in the occupancy grid.
        for i in range(occupancy_grid.shape[0]):
            for j in range(occupancy_grid.shape[1]):
                # If the cell is 0 and has a neighboring cell (representing and obstacle)
                # within a set distance, set it to 100 (obstacle).
                if occupancy_grid[i][j] == 0:
                    for k in range(-distance, distance + 1):
                        for l in range(-distance, distance + 1):
                            # Check the bounds of the occupancy grid.
                            if i + k >= 0 and i + k < occupancy_grid.shape[0] and j + l >= 0 and j + l < occupancy_grid.shape[1]:
                                if occupancy_grid[i + k][j + l] == 100:
                                    new_grid[i][j] = 100
                                    break


        # Search for a path (using breadth first search) in the grid.
        path = bfs(new_grid, start, goal)

        # Return the sequence of poses in that reference frame that the robot should follow.
        return path

    def move(self, linear_velocity, angular_velocity):
        """Send a velocity command (linear velocity in m/s, angular velocity in rad/s)."""
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)
    
    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
    
    def move_to_position(self, x, y):
        """Move the robot to a given position."""
        # Rotation
        # Calculate using arctan and the current orientation of the robot (modulo 2 pi).
        angle = (math.atan2(y - self.y_position, x - self.x_position) - self.orientation) % (2 * math.pi)

        # Continue to rotate until reaching the appropriate rotation.
        while angle > self.angle_tolerance:
            # If the value of the angle is in the interval (0, pi), rotate counterclockwise (positive angular velocity).
            if angle <= math.pi:
                self.move(0.0, self.angular_velocity)
            
            # If the value of the angle is in the interval (-pi, 0), rotate clockwise (negative angular velocity).
            else:
                self.move(0.0, -self.angular_velocity)
            
            # Recalculate the angle, based on the new orientation.
            angle = (math.atan2(y - self.y_position, x - self.x_position) - self.orientation) % (2 * math.pi)
        
        # Translation
        # Calculate the x and y distances.
        x_distance = x - self.x_position
        y_distance = y - self.y_position

        # Calculate the actual (Euclidean) distance using sqrt.
        distance = math.sqrt(x_distance ** 2 + y_distance ** 2)

        # Continue moving until reaching the appropriate location.
        while distance > self.distance_tolerance:
            self.move(self.linear_velocity, 0.0)
            
            # Recalculate the x and y distances, based on the new position.
            x_distance = x - self.x_position
            y_distance = y - self.y_position

            # Recalculate the actual (Euclidean) distance, based on the new position.
            distance = math.sqrt(x_distance ** 2 + y_distance ** 2)

        self.fsm = FSM.STOP

    def run(self):
        """Create a path, by moving the robot to various points."""
        # Determine the translation and rotation of the transform from map to odom.
        (translation, rotation) = self.transform.lookupTransform('/map', '/odom', rospy.Time(0))

        # Determine the yaw by using 'euler_from_quaternion'.
        (_, _, yaw) = euler_from_quaternion([rotation[0], rotation[1], rotation[2], rotation[3]])

        # Find the points (in the 'map' reference frame), by using the appropriate method and conversion.
        map_points = self.find_path((int(translation[0] / self.resolution), int(translation[1] / self.resolution)), 
                                    (int(self.goal_x / self.resolution), int(self.goal_y / self.resolution)))
        
        # Convert the points in the 'map' reference frame to the global reference frame.
        points = []
        for (x, y) in map_points:
            points.append((x * self.resolution, y * self.resolution))
        
        # Assign the path as appropriate, to be used by 'rviz' for the path visualization.
        self.path = points

        # # # # # # # # # #

        rate = rospy.Rate(FREQUENCY) # Loop at 10 Hz.

        while not rospy.is_shutdown():
            # Keep looping until user presses CTRL + C.

            if self.fsm == FSM.MOVE:
                # Cycle through the given points, listed (x1, y1), (x2, y2), etc.
                for (x, y) in points:
                    # Move the robot to the given position (point).
                    self.move_to_position(x - translation[0], y - translation[1])
                                
            else:
                self.stop()

            # Looping at 10 Hz.
            rate.sleep()
        
        self.fsm = FSM.STOP

def main():
    """Main Function"""

    # 1st - Initialize node.
    rospy.init_node("Planning")

    # 2nd - Create an instance of the class with the relevant publishers/subscribers.
    planning = Planning()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(planning.stop)

    # Planning
    try:
        planning.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Node Interrupted")

if __name__ == "__main__":
    """Run the main function."""
    main()
