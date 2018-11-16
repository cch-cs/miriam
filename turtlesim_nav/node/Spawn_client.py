#!/usr/bin/env python

import sys
import rospy
from turtlesim.srv import Spawn

def turtlesim_spawn_client(x, y, theta, name):
    rospy.wait_for_service('spawn')
    try:
        turtlesim_spawn = rospy.ServiceProxy('spawn', Spawn)
        resp1 = turtlesim_spawn(x, y, theta, name)
        print("turlesim spawned", resp1.name)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    x = 3.0
    y = 4.0
    theta = 0.0
    name = "turtle2"
    turtlesim_spawn_client(x,y,theta,name)

