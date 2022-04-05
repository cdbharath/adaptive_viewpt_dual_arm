#!/usr/bin/env python2

# Author: Bharath Kumar Ramesh Babu
# Email: brameshbabu@wpi.edu

import rospy
from rospy.core import loginfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from math import pi
from geometry_msgs.msg import PointStamped


class VisualServoing:
    def __init__(self):
        rospy.init_node('visual_SErvoing')

        self.bridge = CvBridge()
        self.joint_angles = np.array([0, 0, 0, 0, 0, 0, 0])
        self.corners = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])

        # give a proper value
        self.reference_corners = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
        self.image = None

        rospy.Subscriber('/panda_camera/rgb/image_raw', Image, self.image_cb)
        rospy.Subscriber('panda1/joint_states', JointState, self.joint_state_cb)
        
        # subscribe to corners
        rospy.Subscriber('corner1', Float64, self.corner1_cb)
        rospy.Subscriber('corner2', Float64, self.corner2_cb)
        rospy.Subscriber('corner3', Float64, self.corner3_cb)
        rospy.Subscriber('corner4', Float64, self.corner4_cb)

        # TODO change names accordingly
        # TODO change to velocity controller
        pub_q1_vel = rospy.Publisher('/panda1/joint1_velocity_controller/command', Float64, queue_size=10)
        pub_q2_vel = rospy.Publisher('/panda1/joint2_velocity_controller/command', Float64, queue_size=10)
        pub_q3_vel = rospy.Publisher('/panda1/joint3_velocity_controller/command', Float64, queue_size=10)
        pub_q4_vel = rospy.Publisher('/panda1/joint4_velocity_controller/command', Float64, queue_size=10)
        pub_q5_vel = rospy.Publisher('/panda1/joint5_velocity_controller/command', Float64, queue_size=10)
        pub_q6_vel = rospy.Publisher('/panda1/joint6_velocity_controller/command', Float64, queue_size=10)
        pub_q7_vel = rospy.Publisher('/panda1/joint7_velocity_controller/command', Float64, queue_size=10)

        rospy.Timer(rospy.Duration(0.1), self.controller_cb)

    def image_cb(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")

    def joint_state_cb(self, data):
        # TODO check mapping 
        self.joint_angles[0] = data.position[0]
        self.joint_angles[1] = data.position[1]
        self.joint_angles[2] = data.position[2]
        self.joint_angles[3] = data.position[3]
        self.joint_angles[4] = data.position[4]
        self.joint_angles[5] = data.position[5]
        self.joint_angles[6] = data.position[6]

    def corner1_cb(self, data):
        self.corners[0][0] = data.point.x 
        self.corners[0][1] = data.point.y 

    def corner2_cb(self, data):
        self.corners[1][0] = data.point.x 
        self.corners[1][1] = data.point.y 

    def corner3_cb(self, data):
        self.corners[2][0] = data.point.x 
        self.corners[2][1] = data.point.y 

    def corner4_cb(self, data):
        self.corners[3][0] = data.point.x 
        self.corners[3][1] = data.point.y 

    def controller_cb(self, data):
    
        # Calculate transfomation matrices using DH parameters
        # Refer: https://frankaemika.github.io/docs/control_parameters.html 
        # in order: a d alpha theta
        dh_parameters = [[0, 0.333, 0, self.joint_angles[0]],
                         [0, 0, -pi/2, self.joint_angles[1]],
                         [0, 0.316, pi/2, self.joint_angles[2]],
                         [0.0825, 0, pi/2, self.joint_angles[3]],
                         [-0.0825, 0.348, -pi/2, self.joint_angles[4]],
                         [0, 0, pi/2, self.joint_angles[5]],
                         [0.088, 0, pi/2, self.joint_angles[6]]]

        a = []
        transformations = []        
        for dh_parameter in dh_parameters:
            a = dh_parameter[0]
            d = dh_parameter[1]
            alpha = dh_parameter[2]
            theta = dh_parameter[3]

            ai = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                           [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                           [0, np.sin(alpha), np.cos(alpha), d],
                           [0, 0, 0, 1]])
            a.append(ai)
            if len(transformations):
                transformations.append(np.dot(transformations[-1], ai))
            else:
                transformations.append(ai)
        # TODO add camera transformation

        # Calculate feature jacobian
        z = 1
        corner_feature_jacobians = []
        for i, corner in enumerate(self.corners):
            corner_feature_jacobian = np.array([[-1/z, 0, self.corners[i][0]/z, self.corners[i][0]*self.corners[i][1], -(1+self.corners[i][0]*self.corners[i][0]), self.corners[i][1]],
                                                [0, -1/z, self.corners[i][1]/z, 1+self.corners[i][1]*self.corners[i][1], -self.corners[i][0]*self.corners[i][1], -self.corners[i][0]]])

            corner_feature_jacobians.append(corner_feature_jacobian)
        image_jacobian = np.vstack(corner_feature_jacobians)

        # Calculate robot jacobian
        # TODO
        robot_jacobian = None

        # TODO shift control parameters to a good place
        # Control law
        lam = 0.01
        lam_mat = np.eye(6)*lam

        error = np.array([[self.corners[0][0] - self.reference_corners[0][0]],
                          [self.corners[0][1] - self.reference_corners[0][1]],
                          [self.corners[1][0] - self.reference_corners[1][0]],
                          [self.corners[1][1] - self.reference_corners[1][1]],
                          [self.corners[2][0] - self.reference_corners[2][0]],
                          [self.corners[2][1] - self.reference_corners[2][1]],
                          [self.corners[3][0] - self.reference_corners[3][0]],
                          [self.corners[3][1] - self.reference_corners[3][1]]])

        # matrix sizes:
        # error: 8x1
        # image_jacobian: 8x6
        # reference_cartesian_velocities: 6x1  
        # robot_jacobian: 6x7
        # input_joint_velocities: 7x1   
        reference_cartesian_velocities = np.dot(lam_mat, np.dot(np.linalg.pinv(image_jacobian), error))
        input_joint_velocities = np.dot(np.linalg.pinv(robot_jacobian), reference_cartesian_velocities)

        # TODO chech mapping
        pub_q1_vel.publish(input_joint_velocities[0])
        pub_q2_vel.publish(input_joint_velocities[1])
        pub_q3_vel.publish(input_joint_velocities[2])
        pub_q4_vel.publish(input_joint_velocities[3])
        pub_q5_vel.publish(input_joint_velocities[4])
        pub_q6_vel.publish(input_joint_velocities[5])
        pub_q7_vel.publish(input_joint_velocities[6])

if __name__ == "__main__":
    pass
