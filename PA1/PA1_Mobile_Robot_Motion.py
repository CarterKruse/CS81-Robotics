#!/usr/bin/env python

# Author: Carter Kruse
# Date: April 13, 2023

# Import Relevant Libraries
import rospy # Module - ROS APIs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Import Python Modules
import math
import numpy as np

# Constants
# Topic Names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_ODOM_TOPIC = 'odom'

# Velocities
LINEAR_VELOCITY = 0.1 # m/s
ANGULAR_VELOCITY = math.pi / 8 # rad/s

# Tolerances
ANGLE_TOLERANCE = 0.04
DISTANCE_TOLERANCE = 0.12

# Frequency at which the loop operates.
FREQUENCY = 10 # Hz

class MobileRobotMotion():
    """Class - ROS Node"""

    def __init__(self, linear_velocity = LINEAR_VELOCITY, angular_velocity = ANGULAR_VELOCITY, 
                 angle_tolerance = ANGLE_TOLERANCE, distance_tolerance = DISTANCE_TOLERANCE):
        """Initialization Function / Constructor"""

        # Set Up (Publishers / Subscribers)
        # Publisher - Sends velocity commands.
        self.cmd_vel_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size = 1)

        # Subscriber - Recieves messages from the odometer.
        self.odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self.odom_callback)

        # Parameters
        self.linear_velocity = linear_velocity # Constant Linear Velocity
        self.angular_velocity = angular_velocity # Constant Angular Velocity

        # Define the current position and orientation of the robot.
        self.x_position = 0.0
        self.y_position = 0.0
        self.orientation = 0.0

        # Define the angle and position tolerances.
        self.angle_tolerance = angle_tolerance
        self.distance_tolerance = distance_tolerance

        self.no_movement = False

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
    
    def move_to_position(self, x, y, orientation):
        """Move the robot to a given position, with a given orientation."""

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
        
        # Rotation
        # Calculate using the 'goal' orientation and the current orientation of the robot (modulo 2 pi).
        angle = (orientation - self.orientation) % (2 * math.pi)

        # Continue to rotate until reaching the appropriate rotation.
        while angle > self.angle_tolerance:
            # If the value of the angle is in the interval (0, pi), rotate counterclockwise (positive angular velocity).
            if angle <= math.pi:
                self.move(0.0, self.angular_velocity)
            
            # If the value of the angle is in the interval (-pi, 0), rotate clockwise (negative angular velocity).
            else:
                self.move(0.0, -self.angular_velocity)
            
            # Recalculate the angle, based on the new orientation.
            angle = (orientation - self.orientation) % (2 * math.pi)
        
        self.no_movement = True
    
    def move_to_start(self):
        """Move the robot back to the starting position and orientation."""

        self.move_to_position(0, 0, 0)
    
    def move_to_position_simple(self, x, y):
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

        self.no_movement = True
    
    def create_trapezoid(self, start_angle, radius):
        """Move the robot in a trapezoid shape, with a given angle and radius."""

        rate = rospy.Rate(FREQUENCY) # Loop at 10 Hz.

        while not rospy.is_shutdown():
            # Keep looping until user presses CTRL + C.

            if not self.no_movement:
                # Move the robot to the start position/orientation.
                self.move_to_start()

                # Calculate the angle ('theta').
                angle = math.pi / 2 + start_angle

                # Move the robot to the four specified 'corners'.
                self.move_to_position_simple(radius * math.cos(angle), radius * math.sin(angle))
                self.move_to_position_simple(radius * (math.cos(angle - math.pi / 2 + math.pi / 4)), radius * (math.sin(angle - math.pi / 2 + math.pi / 4)))
                self.move_to_position_simple(radius * (math.cos(angle - math.pi / 2 - math.pi / 4)), radius * (math.sin(angle - math.pi / 2 - math.pi / 4)))
                self.move_to_position_simple(radius * -math.cos(angle), radius * -math.sin(angle))

                # Move the robot to the start position/orientation.
                self.move_to_start()

            # Looping at 10 Hz.
            rate.sleep()  

    def create_arc(self, orientation, radius):
        """Move the robot in a circular arc to a given angle, with a given radius."""

        # Calculate using the 'goal' orientation and the current orientation of the robot (modulo 2 pi).
        angle = (orientation - self.orientation) % (2 * math.pi)

        # Continue to rotate until reaching the appropriate rotation.
        while angle > self.angle_tolerance:
            # Use an angular velocity according to v=wr.
            self.move(self.linear_velocity, -self.linear_velocity / radius)
            
            # Recalculate the angle, based on the new orientation.
            angle = (orientation - self.orientation) % (2 * math.pi)
        
        self.no_movement = True
    
    def create_D(self, radius):
        """Move the robot in a D shape, with a given radius."""

        rate = rospy.Rate(FREQUENCY) # Loop at 10 Hz.

        while not rospy.is_shutdown():
            # Keep looping until user presses CTRL + C.

            if not self.no_movement:
                # Move the robot to the start position/orientation.
                self.move_to_start()

                # Move to the top of the D.
                self.move_to_position(0, radius, 0)

                # Create the arc, so that the robot ends facing -pi.
                self.create_arc(-math.pi, radius)

                # Move the robot to the start position/orientation.
                self.move_to_start()

            # Looping at 10 Hz.
            rate.sleep()

    def create_polygon(self, points):
        """Create a polygon, by moving the robot to various points."""

        # Determine the angle ('theta') used in the 2D homogeneous transformation matrix.
            # The origin is on the first point of the list.
            # The x-axis is aligned with the line segment determined by the first two points.
        theta = math.atan2(points[3] - points[1], points[2] - points[0])

        # Determine the 2D homogeneous transformation matrix between the two frames (from local to global).
        local_to_global_transformation_matrix = np.array([[math.cos(theta), -math.sin(theta), 0, 5.5],
                                 [math.sin(theta), math.cos(theta), 0, 5.5],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
        
        print(' ')
        print('Transformation Matrix: \n' + str(local_to_global_transformation_matrix) + '\n')

        # Determine the other 2D homogeneous transformation matrix between the two frames (from global to local).
        global_to_local_transformation_matrix = np.linalg.inv(local_to_global_transformation_matrix)

        print('Points: ')
        # Cycle through the given points, listed (x1, y1, x2, y2, etc).
        for i in range(0, len(points), 2):
            # Print out in the terminal the points of the polyline in the local reference frame.
            result = np.matmul(global_to_local_transformation_matrix, np.array([[points[i]], [points[i + 1]], [0], [1]]))

            print('(' + str(round(result[0][0], 3)) + ', ' + str(round(result[1][0], 3)) + ')')

        rate = rospy.Rate(FREQUENCY) # Loop at 10 Hz.

        while not rospy.is_shutdown():
            # Keep looping until user presses CTRL + C.

            if not self.no_movement:
                # Cycle through the given points, listed (x1, y1, x2, y2, etc).
                for i in range(0, len(points), 2):
                    # Adjust the points to the odom frame (as this is what the implementation uses).
                    shift = 5.5

                    # Move the robot to the given position (point).
                    self.move_to_position_simple(points[i] - shift, points[i + 1] - shift)

            # Looping at 10 Hz.
            rate.sleep()

def main():
    """Main Function"""

    # 1st - Initialize node.
    rospy.init_node("Mobile_Robot_Motion")

    # 2nd - Create an instance of the class with the relevant publishers/subscribers.
    mobile_robot_motion = MobileRobotMotion()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(mobile_robot_motion.stop)

    # Mobile Robot Motion
    try:
        # Trapezoid
        # radius = 2
        # start_angle = math.pi / 6
        # mobile_robot_motion.create_trapezoid(start_angle, radius)
        
        # D Shape
        # radius = 1
        # mobile_robot_motion.create_D(radius)

        # Polygon
        points = [0, 0, 2, 2, 3, -1, 0, -4, 1, -1, -3, -1, -2, 3, 0, 0]
        global_points = []

        for item in points:
            global_points.append(item + 5.5)

        mobile_robot_motion.create_polygon(global_points)

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Node Interrupted")

if __name__ == "__main__":
    """Run the main function."""
    main()
