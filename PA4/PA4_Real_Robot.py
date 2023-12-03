#!/usr/bin/env python

# Author: Carter Kruse
# Date: May 10, 2023

# Import Relevant Libraries
import rospy # Module - ROS APIs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

# Import Python Modules
import math

# Constants
# Topic Names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_ODOM_TOPIC = 'odom'
DEFAULT_SCAN_TOPIC = 'base_scan' # Simulation: 'base_scan', Robot: 'scan'

# Velocities
LINEAR_VELOCITY = 0.1 # m/s
ANGULAR_VELOCITY = math.pi / 8 # rad/s

# Frequency at which the loop operates.
FREQUENCY = 10 # Hz

class RealRobot():
    """Class - ROS Node"""

    def __init__(self, linear_velocity = LINEAR_VELOCITY, angular_velocity = ANGULAR_VELOCITY):
        """Initialization Function / Constructor"""

        # Set Up (Publishers / Subscribers)
        # Publisher - Sends velocity commands.
        self.cmd_vel_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size = 1)

        # Subscriber - Recieves messages from the odometer.
        self.odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self.odom_callback)

        # Subscriber - Receives messages from the laser.
        self.laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self.laser_callback, queue_size = 1)

        # Parameters
        self.linear_velocity = linear_velocity # Constant Linear Velocity
        self.angular_velocity = angular_velocity # Constant Angular Velocity

        # Define the current position and orientation of the robot.
        self.x_position = 0.0
        self.y_position = 0.0
        self.orientation = 0.0

        # Define the current laser scan reading (directly in front of the robot).
        self.laser_scan_distance = 0.0

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

        # Find the index (of the laser scan) corresponding to the measurement directly in front of the robot.
        # Real Robot
        # scan_index = 0 

        # Simulation
        scan_index = len(scan_msg.ranges) / 2

        # From this, determine the distance measurement and set the appropriate variable.
        self.laser_scan_distance = scan_msg.ranges[scan_index]
    
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
        print("Laser Scan Distance")
        print(self.laser_scan_distance)

        print("Odometry")
        print(self.x_position)
        print(self.y_position)
        print(self.orientation)

        self.translate(1)
        # self.rotate_rel(math.pi / 6)
        # self.rotate_rel(-math.pi / 6)

        print("Laser Scan Distance")
        print(self.laser_scan_distance)

        print("Odometry")
        print(self.x_position)
        print(self.y_position)
        print(self.orientation)

def main():
    """Main Function"""

    # 1st - Initialize node.
    rospy.init_node("Real_Robot")

    # 2nd - Create an instance of the class with the relevant publishers/subscribers.
    real_robot = RealRobot()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(real_robot.stop)

    # Real Robot
    try:
        real_robot.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Node Interrupted")

if __name__ == "__main__":
    """Run the main function."""
    main()
