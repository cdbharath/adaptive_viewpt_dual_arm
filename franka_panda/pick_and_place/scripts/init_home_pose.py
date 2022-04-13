#!/usr/bin/env python

import rospy
# from pick_and_place_module.pick_and_place import PickAndPlace
from std_msgs.msg import Float64
from math import pi
import os
from controller_manager_msgs.srv import SwitchController


if __name__ == "__main__":
    rospy.init_node("init_home_pose")
    joint1 = rospy.Publisher("/panda1/panda_joint1_pcontroller/command", Float64, queue_size=10)
    joint2 = rospy.Publisher("/panda1/panda_joint2_pcontroller/command", Float64, queue_size=10)
    joint3 = rospy.Publisher("/panda1/panda_joint3_pcontroller/command", Float64, queue_size=10)
    joint4 = rospy.Publisher("/panda1/panda_joint4_pcontroller/command", Float64, queue_size=10)
    joint5 = rospy.Publisher("/panda1/panda_joint5_pcontroller/command", Float64, queue_size=10)
    joint6 = rospy.Publisher("/panda1/panda_joint6_pcontroller/command", Float64, queue_size=10)
    joint7 = rospy.Publisher("/panda1/panda_joint7_pcontroller/command", Float64, queue_size=10)      
    rospy.sleep(1)

    rospy.wait_for_service('/panda1/controller_manager/switch_controller')
    try:
        sc_service = rospy.ServiceProxy('/panda1/controller_manager/switch_controller', SwitchController)
        stop_controllers = ['panda_joint1_controller', 'panda_joint2_controller', 'panda_joint3_controller', 'panda_joint4_controller', 'panda_joint5_controller', 'panda_joint6_controller', 'panda_joint7_controller']
        start_controllers = ['panda_joint1_pcontroller', 'panda_joint2_pcontroller', 'panda_joint3_pcontroller', 'panda_joint4_pcontroller', 'panda_joint5_pcontroller', 'panda_joint6_pcontroller', 'panda_joint7_pcontroller']
        strictness = 2
        start_asap = False
        timeout = 0.0
        res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
        rospy.loginfo('switching successful')
    except rospy.ServiceException as e:
        rospy.loginfo("Service Call Failed")	


    joint1.publish(-1.2855552287447114)
    joint2.publish(-0.8900155883125755)
    joint3.publish(1.1781403402713089)
    joint4.publish(-2.037701347149067)
    joint5.publish(0.801004954856599)
    joint6.publish(1.5885867163382619)
    joint7.publish(0.17028892513511007)
    rospy.sleep(1)

    rospy.wait_for_service('/panda1/controller_manager/switch_controller')
    try:
        sc_service = rospy.ServiceProxy('/panda1/controller_manager/switch_controller', SwitchController)
        stop_controllers = ['panda_joint1_pcontroller', 'panda_joint2_pcontroller', 'panda_joint3_pcontroller', 'panda_joint4_pcontroller', 'panda_joint5_pcontroller', 'panda_joint6_pcontroller', 'panda_joint7_pcontroller']
        start_controllers = ['panda_joint1_controller', 'panda_joint2_controller', 'panda_joint3_controller', 'panda_joint4_controller', 'panda_joint5_controller', 'panda_joint6_controller', 'panda_joint7_controller']
        strictness = 2
        start_asap = False
        timeout = 0.0
        res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
        rospy.loginfo('switching successful')
    except rospy.ServiceException as e:
        rospy.loginfo("Service Call Failed")	


    rospy.spin()
