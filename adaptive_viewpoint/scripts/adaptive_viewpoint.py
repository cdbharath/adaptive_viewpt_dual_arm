#!/usr/bin/env python

# Author: Bharath Kumar Ramesh Babu, Akshay Kumar Harikrishnan, Harishkumar Ramdhas
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
        rospy.init_node('visual_Servoing')

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

        pub_q1_vel = rospy.Publisher('/panda1/panda_joint1_controller/command', Float64, queue_size=10)
        pub_q2_vel = rospy.Publisher('/panda1/panda_joint2_controller/command', Float64, queue_size=10)
        pub_q3_vel = rospy.Publisher('/panda1/panda_joint3_controller/command', Float64, queue_size=10)
        pub_q4_vel = rospy.Publisher('/panda1/panda_joint4_controller/command', Float64, queue_size=10)
        pub_q5_vel = rospy.Publisher('/panda1/panda_joint5_controller/command', Float64, queue_size=10)
        pub_q6_vel = rospy.Publisher('/panda1/panda_joint6_controller/command', Float64, queue_size=10)
        pub_q7_vel = rospy.Publisher('/panda1/panda_joint7_controller/command', Float64, queue_size=10)

        rospy.Timer(rospy.Duration(0.1), self.controller_cb)

    def image_cb(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")

    def joint_state_cb(self, data):
        self.joint_angles[0] = data.position[2]
        self.joint_angles[1] = data.position[3]
        self.joint_angles[2] = data.position[4]
        self.joint_angles[3] = data.position[5]
        self.joint_angles[4] = data.position[6]
        self.joint_angles[5] = data.position[7]
        self.joint_angles[6] = data.position[8]

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
            a, d, alpha, theta = dh_parameter[0], dh_parameter[1], dh_parameter[2], dh_parameter[3] 

            ai = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                           [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                           [0, np.sin(alpha), np.cos(alpha), d],
                           [0, 0, 0, 1]])
            a.append(ai)
            if len(transformations):
                transformations.append(np.dot(transformations[-1], ai))
            else:
                transformations.append(ai)
        
        # Camera transformation
        # Transformations in order xyzrpy
        camera_xyzrpy = [[0.05, -0.05, 0, -1.57, -1.57, -0.789], 
                         [0, 0, 0, -1.57, 0, -1.57]]
        
        for xyzrpy in camera_xyzrpy:
            x, y, z, R, P, Y = xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]
            cam_t = np.array([[np.cos(Y)*np.cos(Z), np.sin(X)*np.sin(Y)*np.cos(Z) - np.cos(X)*np.sin(Z), np.cos(X)*np.sin(Y)*np.cos(Z) + np.sin(X)*np.sin(Z), x],
                              [np.cos(Y)*np.sin(Z), np.sin(X)*np.sin(Y)*np.sin(Z) + np.cos(X)*np.cos(Z), np.cos(X)*np.sin(Y)*np.sin(Z) - np.sin(X)*np.cos(Z), y],
                              [-np.sin(Y), np.sin(X)*np.cos(Y), np.cos(X)*np.cos(Y), z],
                              [0, 0, 0, 1]])

            a.append(cam_t)
            transformations.append(np.dot(transformations[-1], cam_t))
        
        # Calculate feature jacobian
        z = 1
        corner_feature_jacobians = []
        for i, corner in enumerate(self.corners):
            corner_feature_jacobian = np.array([[-1/z, 0, self.corners[i][0]/z, self.corners[i][0]*self.corners[i][1], -(1+self.corners[i][0]*self.corners[i][0]), self.corners[i][1]],
                                                [0, -1/z, self.corners[i][1]/z, 1+self.corners[i][1]*self.corners[i][1], -self.corners[i][0]*self.corners[i][1], -self.corners[i][0]]])

            corner_feature_jacobians.append(corner_feature_jacobian)
        image_jacobian = np.vstack(corner_feature_jacobians)

        # Calculate robot jacobian
        robot_jacobian = None

        for i in range(len(dh_parameters)):
            if i == 0:
                joint_av = np.array([[0],[0],[1]])
                joint_lv = np.cross(np.array([[0],[0],[1]]), np.array([[transformations[len(dh_parameters) - 1][0][3]],[transformations[len(dh_parameters) - 1][1][3]],[transformations[len(dh_parameters) - 1][2][3]]]), axis=0)
                
                joint_jacobain = np.vstack((joint_lv,joint_av))

            else:                                
                joint_av = np.array([[transformations[i][0][3]],[transformations[i][1][3]],[transformations[i][2][3]]])
                joint_lv = np.cross(np.array([[transformations[i][0][3]],[transformations[i][1][3]],[transformations[i][2][3]]]), np.array([[transformations[len(dh_parameters) - 1][0][3]],[transformations[len(dh_parameters) - 1][1][3]],[v[2][3]]]) - np.array([[transformations[i][0][3]],[transformations[i][1][3]],[transformations[i][2][3]]]), axis=0)

                joint_jacobain = np.vstack((joint_lv,joint_av))
                robot_jacobian = np.hstack((robot_jacobian, joint_jacobain))

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

        pub_q1_vel.publish(input_joint_velocities[0])
        pub_q2_vel.publish(input_joint_velocities[1])
        pub_q3_vel.publish(input_joint_velocities[2])
        pub_q4_vel.publish(input_joint_velocities[3])
        pub_q5_vel.publish(input_joint_velocities[4])
        pub_q6_vel.publish(input_joint_velocities[5])
        pub_q7_vel.publish(input_joint_velocities[6])

if __name__ == "__main__":
    pass
