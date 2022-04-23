#!/usr/bin/env python3

import rospy
import sys
from controller_manager_msgs.srv import SwitchController


if __name__ == "__main__":
        rospy.init_node('detect_features_node')
        rospy.loginfo('switching to %s controller', sys.argv[1])
        
        if(sys.argv[1]=='position'):
            rospy.wait_for_service('/vbmbot/controller_manager/switch_controller')
            try:
                sc_service = rospy.ServiceProxy('/vbmbot/controller_manager/switch_controller', SwitchController)
                stop_controllers = ['joint1_velocity_controller','joint2_velocity_controller']
                start_controllers = ['joint1_position_controller','joint2_position_controller']
                strictness = 2
                start_asap = False
                timeout = 0.0
                res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
                rospy.loginfo('switching successful')
            except rospy.ServiceException as e:
                rospy.loginfo("Service Call Failed")	

        if(sys.argv[1]=='velocity'):
            rospy.wait_for_service('/vbmbot/controller_manager/switch_controller')
            try:
                sc_service = rospy.ServiceProxy('/vbmbot/controller_manager/switch_controller', SwitchController)
                start_controllers = ['joint1_velocity_controller','joint2_velocity_controller']
                stop_controllers = ['joint1_position_controller','joint2_position_controller']
                strictness = 2
                start_asap = False
                timeout = 0.0
                res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
                rospy.loginfo('switching successful')
            except rospy.ServiceException as e:
                rospy.loginfo("Service Call Failed")	
        
        rospy.spin()        
