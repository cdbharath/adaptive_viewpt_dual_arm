#!/usr/bin/env python

import rospy
# from pick_and_place_module.pick_and_place import PickAndPlace
from std_msgs.msg import Float64
from math import pi
import os

if __name__ == "__main__":
    rospy.init_node("init_home_pose")
    joint1 = rospy.Publisher("/panda1/panda_joint1_controller/command", Float64, queue_size=10)
    joint2 = rospy.Publisher("/panda1/panda_joint2_controller/command", Float64, queue_size=10)
    joint3 = rospy.Publisher("/panda1/panda_joint3_controller/command", Float64, queue_size=10)
    joint4 = rospy.Publisher("/panda1/panda_joint4_controller/command", Float64, queue_size=10)
    joint5 = rospy.Publisher("/panda1/panda_joint5_controller/command", Float64, queue_size=10)
    joint6 = rospy.Publisher("/panda1/panda_joint6_controller/command", Float64, queue_size=10)
    joint7 = rospy.Publisher("/panda1/panda_joint7_controller/command", Float64, queue_size=10)      
    rospy.sleep(1)

    joint1.publish(-1.5855552287447114)
    joint2.publish(-0.8900155883125755)
    joint3.publish(1.1781403402713089)
    joint4.publish(-2.037701347149067)
    joint5.publish(0.801004954856599)
    joint6.publish(1.5885867163382619)
    joint7.publish(0.17028892513511007)
    
