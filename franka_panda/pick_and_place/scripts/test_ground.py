#!/usr/bin/env python

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace
from std_msgs.msg import Float64
from math import pi
import os

os.environ["ROS_NAMESPACE"] = "panda2"

def debug():
    pick_and_place = PickAndPlace(0.05, 0.4)
    
    pick_and_place.setPickPose(0.3, 0.0, 0.12, 0, pi, 0)
    pick_and_place.setDropPose(0.0, 0.4, 0.4, 0, pi, 0)
    pick_and_place.setGripperPose(0.01, 0.005)
    
    # pick_and_place.execute_pick_and_place()
    # pick_and_place.execute_cartesian_pick_and_place()
    # pick_and_place.execute_cartesian_pick_up()
    pick_and_place.execute_pick_up()

if __name__ == "__main__":
    debug()