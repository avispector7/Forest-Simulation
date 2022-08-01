#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def controller(lin, ang, pub, rate):
    speed = Twist()
    speed.linear.x = lin
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = ang

    #rospy.loginfo(speed)
    pub.publish(speed)
    rate.sleep()

def drive(distance, pub, rate):
    if distance < 0:
        direction = -1
        distance = abs(distance)
    else:
        direction = 1

    start = time.time()
    while time.time() <= start + distance:
        controller(direction, 0, pub, rate)

def turn_deg(degrees, pub, rate):
    if degrees < 0:
        direction = -1
        degrees = abs(degrees)
    else:
        direction = 1

    radians = degrees * 3.14159 / 180
    start = time.time()
    while time.time() <= start + radians:
        controller(0, direction, pub, rate)

def turn_rad(radians, pub, rate):
    if radians < 0:
        direction = -1
        radians = abs(radians)
    else:
        direction = 1
    
    start = time.time()
    while time.time() <= start + radians:
        controller(0, direction, pub, rate)

def main():
    pub = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=0)
    rospy.init_node('controller')
    rate = rospy.Rate(10)

    # controls
    # movement commands here:
    # drive(pos/neg_distance, pub, rate)
    # turn_deg(angle_in_degrees, pub, rate)
    # turn_rad(angle_in_radians, pub, rate)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
