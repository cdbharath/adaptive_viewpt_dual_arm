#!/usr/bin/env python

# Author: Bharath Kumar Ramesh Babu, Akshay Kumar Harikrishnan, Harishkumar Ramdhas
# Email: brameshbabu@wpi.edu


import rospy
import sys
from controller_manager_msgs.srv import SwitchController


if __name__ == "__main__":
        rospy.init_node('init_pose')
        rospy.loginfo('switching to %s controller', sys.argv[1])
        
        if(sys.argv[1]=='velocity'):
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

        if(sys.argv[1]=='position'):
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
        
        rospy.spin()        