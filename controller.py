#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

# controller method
def controller(lin, ang, pub, rate):
    # initializes velocity method and sets components based on input
    velocity = Twist()
    velocity.linear.x = lin
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = ang
    
    # uncomment the following line to print velocity command ROS log
    #rospy.loginfo(velocity)
    
    # publishes velocity to ROS and pauses ROS for the length of rate
    pub.publish(velocity)
    rate.sleep()

# forward drive method - takes in linear distance, ROS publisher, publishing rate
def drive(distance, pub, rate):
    # normalizes distance if negative, sets direction to forward (1) or backward (-1)
    if distance < 0:
        direction = -1
        distance = abs(distance)
    else:
        direction = 1
    
    # calls controller with linear velocity of 1 unit/s, angular velocity of 0
    # loops for as many seconds as distance
    start = time.time()
    while time.time() <= start + distance:
        controller(direction, 0, pub, rate)

# degree rotate method - takes in angle in degrees, ROS publisher, publishing rate
def turn_deg(degrees, pub, rate):
    # normalizes angle if negative, sets direction
    if degrees < 0:
        direction = -1
        degrees = abs(degrees)
    else:
        direction = 1
    
    # converts degrees to radians
    radians = degrees * 3.14159 / 180
    
    # calls controller with linear velocity of 0, angular velocity of 1 radian/s
    # loops for as many seconds as radians
    start = time.time()
    while time.time() <= start + radians:
        controller(0, direction, pub, rate)

# radian rotate method - takes in angle in radians, ROS publisher, publishing rate
def turn_rad(radians, pub, rate):
    # normalizes angle if negative, sets direction
    if radians < 0:
        direction = -1
        radians = abs(radians)
    else:
        direction = 1
    
    # calls controller with linear velocity of 0, angular velocity of 1 radian/s
    # loops for as many seconds as radians
    start = time.time()
    while time.time() <= start + radians:
        controller(0, direction, pub, rate)

def main():
    # creates a publisher to the Husky velocity command topic that takes in Twist messages
    pub = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=0)
    
    # initializes a ROS node
    rospy.init_node('controller')
    
    # sets the rate at which to publish methods
    rate = rospy.Rate(10)

    # Controls:
    
    # Drive:
    # drive(distance, pub, rate)
    # distance - straight distance to travel, positive for forward, negative for backward
    # pub, rate from above
    
    # Turn:
    # turn_deg(angle_in_degrees, pub, rate)
    # turn_rad(angle_in_radians, pub, rate)
    # angle - angle to turn, standard position - positive is counterclockwise
    # call turn_deg to input an angle in degrees, turn_rad to input an angle in radians
    # pub, rate from above
    
    # movement commands here:
    

# calls the main() method at run
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
