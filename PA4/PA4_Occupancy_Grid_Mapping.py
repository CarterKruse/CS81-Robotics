#!/usr/bin/env python

# Author: Carter Kruse
# Date: May 10, 2023

# Import Relevant Libraries
import rospy # Module - ROS APIs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion

# Import Python Modules
import math
import numpy as np

# Constants
# Topic Names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_ODOM_TOPIC = 'odom'
DEFAULT_SCAN_TOPIC = 'base_scan' # Simulation: 'base_scan', Robot: 'scan'
DEFAULT_MAP_TOPIC = 'map'

# Velocities
LINEAR_VELOCITY = 0.1 # m/s
ANGULAR_VELOCITY = math.pi / 8 # rad/s

# Occupancy Grid
WIDTH = 400
HEIGHT = 400

# Real Robot
MAPPING_TIME = 20 # s

# Frequency at which the loop operates.
FREQUENCY = 10 # Hz

class OccupancyGridMapping():
    """Class - ROS Node"""

    def __init__(self, linear_velocity = LINEAR_VELOCITY, angular_velocity = ANGULAR_VELOCITY,
                 width = WIDTH, height = HEIGHT):
        """Initialization Function / Constructor"""

        # Set Up (Publishers / Subscribers)
        # Publisher - Sends velocity commands.
        self.cmd_vel_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size = 1)

        # Publisher - Sends occupancy grid results.
        self.map_pub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size = 1)

        # Subscriber - Recieves messages from the odometer.
        self.odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self.odom_callback)

        # Subscriber - Receives messages from the laser.
        self.laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self.laser_callback, queue_size = 1)

        # Parameters
        self.linear_velocity = linear_velocity # Constant Linear Velocity
        self.angular_velocity = angular_velocity # Constant Angular Velocity

        # Occupancy Grid Info
        self.resolution = 0.05
        self.width = width
        self.height = height
        
        # Occupancy Grid Map
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'odom'
        self.map.info.resolution = self.resolution
        self.map.info.width = self.width
        self.map.info.height = self.height
        self.map.info.origin.position.x = 10
        self.map.info.origin.position.y = 10
        self.map.data = [-1] * self.width * self.height

        # Define the current position and orientation of the robot.
        self.x_position = 0.0
        self.y_position = 0.0
        self.orientation = 0.0

        # Wait for a few seconds for the registration (to ROS master).
        rospy.sleep(2.0)
    
    def odom_callback(self, odom_msg):
        """Set the position and location of the robot."""

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

    def laser_callback(self, scan_msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # LaserScan Message http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

        # Convert the laser scan message to a list of ranges.
        ranges = list(scan_msg.ranges)

        # Update the occupancy grid map, by looping through the measurements.
        for i, range in enumerate(ranges):
            # Check to see if the range is out of bounds.
            if range >= scan_msg.range_max:
                continue

            # Create a range of values up to the measurement (for free space locations).
            space_list = np.arange(0.0, range, self.resolution)
            
            for space_range in space_list:
                # Calculate the position of the free space, according to the 'odom' reference frame.
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = self.x_position + space_range * math.cos(self.orientation + angle)
                y = self.y_position + space_range * math.sin(self.orientation + angle)

                # Convert the physical position to grid coordinates.
                x_grid = int(round((x + self.map.info.origin.position.x) / self.resolution))
                y_grid = int(round((y + self.map.info.origin.position.y) / self.resolution))

                # Update the the occupancy grid accordingly.
                if x_grid >= 0 and x_grid < self.width and y_grid >= 0 and y_grid < self.height:
                    # Determine the appropriate index for the array.
                    index = y_grid * self.width + x_grid

                    # Check to make sure the grid point was not already marked as an obstacle.
                    if self.map.data[index] != 100:
                        self.map.data[index] = 0

            # Calculate the position of the obstacle, according to the 'odom' reference frame.
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = self.x_position + range * math.cos(self.orientation + angle)
            y = self.y_position + range * math.sin(self.orientation + angle)

            # Convert the physical position to grid coordinates.
            x_grid = int(round((x + self.map.info.origin.position.x) / self.resolution))
            y_grid = int(round((y + self.map.info.origin.position.y) / self.resolution))

            # Update the OccupancyGrid, accordingly.
            if x_grid >= 0 and x_grid < self.width and y_grid >= 0 and y_grid < self.height:
                # Determine the appropriate index for the array.
                index = y_grid * self.width + x_grid
                self.map.data[index] = 100
        
        # Publish the map information/data to the appropriate topic.
        self.map_pub.publish(self.map)

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist_msg)
    
    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
    
    def translate(self, distance):
        """Translate the robot by a given distance, according to the parameter."""

        rate = rospy.Rate(FREQUENCY) # Loop at 10 Hz.

        # Determine the duration of the translation of the robot.
        duration = abs(distance) / self.linear_velocity

        start_time = rospy.get_rostime()

        while not rospy.is_shutdown():
            # Keep looping until user presses CTRL + C.

            # Check if the appropriate amount of time has passed.
            if rospy.get_rostime() - start_time >= rospy.Duration(duration):
                self.move(0.0, 0.0)
                break

            # Determine the direction of translation.
            if distance >= 0:
                self.move(self.linear_velocity, 0.0)
            
            else:
                self.move(-self.linear_velocity, 0.0)

            # Looping at 10 Hz.
            rate.sleep()
    
    def rotate_rel(self, angle):
        """Rotate the robot by a given angle, according to the parameter."""

        rate = rospy.Rate(FREQUENCY) # Loop at 10 Hz.

        # Determine the duration of the rotation of the robot.
        duration = abs(angle) / self.angular_velocity

        start_time = rospy.get_rostime()

        while not rospy.is_shutdown():
            # Keep looping until user presses CTRL + C.

            # Check if the appropriate amount of time has passed.
            if rospy.get_rostime() - start_time >= rospy.Duration(duration):
                self.move(0.0, 0.0)
                break

            else:
                # Determine the direction of rotation.
                if angle >= 0:
                    self.move(0, self.angular_velocity)
                
                else:
                    self.move(0, -self.angular_velocity)

            # Looping at 10 Hz.
            rate.sleep()
    
    def rotate_abs(self, angle):
        """Rotate the robot to an absolute angle, according to the parameter."""

        # Calculate the relative rotation using the current orientation of the robot (modulo 2 pi).
        relative_angle = (angle - self.orientation) % (2 * math.pi)

        # Modify the relative angle so that the shortest rotation is taken.
        if relative_angle >= math.pi:
            relative_angle = -((2 * math.pi) - relative_angle)

        # Call the previous method/function.
        self.rotate_rel(relative_angle)

    def square(self, distance):
        """Make the robot follow a square, according to the distance parameter."""
        self.translate(distance)
        self.rotate_rel(math.pi / 2)
        self.translate(distance)
        self.rotate_rel(math.pi / 2)
        self.translate(distance)
        self.rotate_rel(math.pi / 2)
        self.translate(distance)
        self.rotate_abs(math.pi / 2)

    def square(self, distance, initial_angle):
        """Make the robot follow a square, according to the distance parameter, returning to a given initial angle."""
        self.translate(distance)
        self.rotate_rel(math.pi / 2)
        self.translate(distance)
        self.rotate_rel(math.pi / 2)
        self.translate(distance)
        self.rotate_rel(math.pi / 2)
        self.translate(distance)
        self.rotate_abs(initial_angle)
    
    def run(self):
        # Real Robot
        # start_time = rospy.get_rostime()

        # while rospy.get_rostime() - start_time < rospy.Duration(MAPPING_TIME):
        #     continue

        # Simulation
        self.translate(1)
        self.rotate_rel(math.pi / 2)
        self.translate(1)
        self.rotate_rel(-math.pi / 2)
        self.translate(5)
        self.rotate_rel(math.pi / 2)
        self.translate(4)
        self.rotate_rel(math.pi / 2)
        self.translate(3)

        # Real Robot & Simulation
        # Create a new array based on the map data, for visualization.
        array = np.array(self.map.data).reshape(self.width, self.height)

        # Reverse to the array type, rather than 'numpy' formatting.
        data = np.asarray(array)

        # Save the data to a CSV file, for later viewing.
        np.savetxt('data.csv', data, delimiter = ',')

def main():
    """Main Function"""

    # 1st - Initialize node.
    rospy.init_node("Occupancy_Grid_Mapping")

    # 2nd - Create an instance of the class with the relevant publishers/subscribers.
    occupancy_grid_mapping = OccupancyGridMapping()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(occupancy_grid_mapping.stop)

    # Occupancy Grid Mapping
    try:
        occupancy_grid_mapping.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Node Interrupted")

if __name__ == "__main__":
    """Run the main function."""
    main()
