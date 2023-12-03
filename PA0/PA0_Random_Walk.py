#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Author: Carter Kruse
# Date: April 3, 2023

# Import Python Modules
import math
import random

# Import Relevant Librarires
import rospy # Module - ROS APIs
from geometry_msgs.msg import Twist # Message Type (cmd_vel)
from sensor_msgs.msg import LaserScan # Message Type (scan)

# Constants
# Topic Names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan'

# Frequency at which the loop operates.
FREQUENCY = 10 # Hz

# Velocities
LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold - Minimum Clearance Distance (< range_max)
MIN_THRESHOLD_DISTANCE = 0.5 # m

# Field of View (Radians)
MIN_SCAN_ANGLE_RAD = -60.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = +60.0 / 180 * math.pi

class RandomWalk():
    def __init__(self, linear_velocity = LINEAR_VELOCITY, angular_velocity = ANGULAR_VELOCITY, 
                 min_threshold_distance = MIN_THRESHOLD_DISTANCE, scan_angle = [MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD]):
        """Constructor"""
        # Set Up (Publishers/Subscribers)
        # Publisher - Sends velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size = 1)
        
        # Subscriber - Receives messages from the laser.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        # Parameters
        self.linear_velocity = linear_velocity # Constant Linear Velocity
        self.angular_velocity = angular_velocity # Constant Angular Velocity
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle
        
        # Flag used to control the behavior of the robot (true if there is a close obstacle).
        self._close_obstacle = False
    
    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting Velocities
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        
        if not self._close_obstacle:
            # Find the min_index and max_index. From this, find the minimum range value between min_scan_angle and max_scan_angle.
            # If the minimum range value found is closer to min_threshold_distance, change the flag self._close_obstacle.
            # LaserScan Message http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

            ####### TODO: ANSWER CODE BEGIN #######

            # Set the minimum distance to the nearest obstacle to the maximum range of the sensor.
            min_distance = msg.range_max

            # Determine the middle index according to the total number of indices (the length of msg.ranges).
            mid_index = len(msg.ranges) / 2

            # Determine the minimum and maximum indices of the LaserScan corresponding to the minimum and maximum scan angles.
            # (MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD)

            min_index = int(mid_index + self.scan_angle[0] / msg.angle_increment)
            max_index = int(mid_index + self.scan_angle[1] / msg.angle_increment)

            # Cycle through min_index to max_index to determine the minimum range value between the minimum and maximum scan angles.
            for i in range(min_index, max_index + 1):
                # If a given range value is less than the current minimum, the minmum is reset.
                if msg.ranges[i] < min_distance and msg.range_min < msg.ranges[i] < msg.range_max:
                    min_distance = msg.ranges[i]
                
                # If the minimum range value is less than the threshold value, the flag is set.
                if min_distance < self.min_threshold_distance:
                    self._close_obstacle = True

            ####### ANSWER CODE END #######

    def spin(self):
        rate = rospy.Rate(FREQUENCY) # Loop at 10 Hz.
        while not rospy.is_shutdown():
            # Keep looping until user presses CTRL + C.

            # If the flag self._close_obstacle is false, the robot should move forward.
            # Otherwise, the robot should rotate for a random angle, after which the flag is set again to false.
            # Use the function move() already implemented, passing the default velocities saved in the corresponding class members.

            ####### TODO: ANSWER CODE BEGIN #######

            # Check if the flag is not set.
            if not self._close_obstacle:
                # Call the move() method with appropriate linear velocity and an angular velocity of zero.
                self.move(self.linear_velocity, 0)

            # Otherwise, if the flag is set...
            else:
                # A random angle is determined using the random.uniform() method.
                angle = random.uniform(-math.pi, math.pi)

                # Convert this into the appropriate duration, according to the angular velocity.
                duration = abs(angle / self.angular_velocity)

                # The duration specifies the length of time that the robot should rotate with a given angular velocity to reach the desired angle.
                start_time = rospy.get_rostime()
                while rospy.get_rostime() - start_time <= rospy.Duration(duration):
                    # If the value of the angle is in the interval (0, pi), it rotates counterclockwise (positive angular velocity).
                    if angle > 0:
                        self.move(0, self.angular_velocity)
                    
                    # If the value of the angle is in the interval (-pi, 0), it rotates clockwise (negative angular velocity).
                    else:
                        self.move(0, -self.angular_velocity)
                
                # After the robot has finished rotating, the flag is unset so that the robot moves forward.
                self._close_obstacle = False
            
            # Looping at 10 Hz.
            rate.sleep()
            
            ####### ANSWER CODE END #######

def main():
    """Main Function"""

    # 1st Initialization Of Node
    rospy.init_node("Random_Walk")

    # Wait for a few seconds for the registration.
    rospy.sleep(2)

    # Initialization Of Class (Random Walk)
    random_walk = RandomWalk()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(random_walk.stop)

    # Robot - Random Walks
    try:
        random_walk.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Node Interrupted")

if __name__ == "__main__":
    """Run the main function."""
    main()
