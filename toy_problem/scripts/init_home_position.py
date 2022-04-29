#!/usr/bin/env python3

import rospy
import sys
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Float64

if __name__ == "__main__":
        rospy.init_node('init_home_position')
        joint1p = rospy.Publisher('/vbmbot/joint1_position_controller/command', Float64, queue_size=10)
        joint2p = rospy.Publisher('/vbmbot/joint2_position_controller/command', Float64, queue_size=10)
        rospy.sleep(1)

        rospy.wait_for_service('/vbmbot/controller_manager/switch_controller')
        try:
            sc_service = rospy.ServiceProxy('/vbmbot/controller_manager/switch_controller', SwitchController)
            stop_controllers = ['joint1_velocity_controller','joint2_velocity_controller']
            start_controllers = ['joint1_position_controller','joint2_position_controller']
            strictness = 2
            start_asap = False
            timeout = 0.0
            res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
            rospy.loginfo('switching position controllers successful')
        except rospy.ServiceException as e:
            rospy.loginfo("Service Call Failed")	

        # case 1

        # joint1p.publish(0.67)
        # joint2p.publish(-1.17)

        # joint1p.publish(-1.27)
        # joint2p.publish(1.47)

        # case 2
         
        # joint1p.publish(0.5)
        # joint2p.publish(-0.2)
        # rospy.sleep(5)

        joint1p.publish(-0.7)
        joint2p.publish(1.47)
        rospy.sleep(2.5)

        joint1p.publish(-1.07)
        joint2p.publish(1.20)
        rospy.sleep(5)

        # joint1p.publish(-1.27)
        # joint2p.publish(1.47)
        # rospy.sleep(5)

        rospy.wait_for_service('/vbmbot/controller_manager/switch_controller')
        try:
            sc_service = rospy.ServiceProxy('/vbmbot/controller_manager/switch_controller', SwitchController)
            start_controllers = ['joint1_velocity_controller','joint2_velocity_controller']
            stop_controllers = ['joint1_position_controller','joint2_position_controller']
            strictness = 2
            start_asap = False
            timeout = 0.0
            res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
            rospy.loginfo('switching velocity controller successful')
        except rospy.ServiceException as e:
            rospy.loginfo("Service Call Failed")	
        
        # rospy.spin()        
