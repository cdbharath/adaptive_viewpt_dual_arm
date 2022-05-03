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
from core import JacobianBody, JacobianSpace
import moveit_commander
import moveit_msgs.msg
import sys

class VisualServoing:
    def __init__(self):
        rospy.init_node('visual_Servoing')

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander(robot_description = "/panda1/robot_description", ns = "panda1")
        scene = moveit_commander.PlanningSceneInterface(ns = "panda1")
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name, robot_description = "/panda1/robot_description", ns = "panda1")

        self.robot = robot
        self.move_group = move_group
        self.bridge = CvBridge()
        self.joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.corners = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])

        # give a proper value
        self.reference_corners = np.array([[426.0, 311.0], [416.0, 277.0], [450.0, 267.0], [460.0, 300.0]])
        self.image = None

        rospy.Subscriber('/panda_camera/rgb/image_raw', Image, self.image_cb)
        rospy.Subscriber('panda1/joint_states', JointState, self.joint_state_cb)
        
        # subscribe to corners
        rospy.Subscriber('/aruco_single/corner1', PointStamped, self.corner1_cb)
        rospy.Subscriber('/aruco_single/corner2', PointStamped, self.corner2_cb)
        rospy.Subscriber('/aruco_single/corner3', PointStamped, self.corner3_cb)
        rospy.Subscriber('/aruco_single/corner4', PointStamped, self.corner4_cb)

        self.vis_pub = rospy.Publisher('visualizer', Image, queue_size=10)

        self.pub_q1_vel = rospy.Publisher('/panda1/panda_joint1_controller/command', Float64, queue_size=10)
        self.pub_q2_vel = rospy.Publisher('/panda1/panda_joint2_controller/command', Float64, queue_size=10)
        self.pub_q3_vel = rospy.Publisher('/panda1/panda_joint3_controller/command', Float64, queue_size=10)
        self.pub_q4_vel = rospy.Publisher('/panda1/panda_joint4_controller/command', Float64, queue_size=10)
        self.pub_q5_vel = rospy.Publisher('/panda1/panda_joint5_controller/command', Float64, queue_size=10)
        self.pub_q6_vel = rospy.Publisher('/panda1/panda_joint6_controller/command', Float64, queue_size=10)
        self.pub_q7_vel = rospy.Publisher('/panda1/panda_joint7_controller/command', Float64, queue_size=10)

        rospy.Timer(rospy.Duration(0.1), self.controller_cb)

        rospy.logerr("Visual Servoing Initialized")

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

    def visualizer(self, image, references):
        visual = image.copy()
        self.vis_pub(self.bridge.cv2_to_imgmsg(visual))

    def controller_cb(self, data):
    
        # Calculate transfomation matrices using DH parameters
        # Refer: https://frankaemika.github.io/docs/control_parameters.html 
        # in order: a d alpha theta
        # dh_parameters = [[0,      0.333, 0,     self.joint_angles[0]],
        #                  [0,      0,     -pi/2, self.joint_angles[1]],
        #                  [0,      0.316, pi/2,  self.joint_angles[2]],
        #                  [0.0825, 0,     pi/2,  self.joint_angles[3]],
        #                  [-0.0825,0.348, -pi/2, self.joint_angles[4]],
        #                  [0,      0,     pi/2,  self.joint_angles[5]],
        #                  [0.088,  0,     pi/2,  self.joint_angles[6]]]

        # ais = []
        # transformations = []        
        # for dh_parameter in dh_parameters:
        #     a, d, alpha, theta = dh_parameter[0], dh_parameter[1], dh_parameter[2], dh_parameter[3] 

        #     ai = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        #                    [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        #                    [0, np.sin(alpha), np.cos(alpha), d],
        #                    [0, 0, 0, 1]])
        #     ais.append(ai)
        #     if len(transformations):
        #         transformations.append(np.dot(transformations[-1], ai))
        #     else:
        #         transformations.append(ai)
        
        # Camera transformation
        # Transformations in order xyzrpy
        # camera_xyzrpy = [[0.06, 0.00, 0, -1.57, -1.57, -0.0], 
        #                  [0, 0, 0, -1.57, 0, -1.57]]
        
        # for xyzrpy in camera_xyzrpy:
        #     x, y, z, X, Y, Z = xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]
        #     cam_t = np.array([[np.cos(Y)*np.cos(Z), np.sin(X)*np.sin(Y)*np.cos(Z) - np.cos(X)*np.sin(Z), np.cos(X)*np.sin(Y)*np.cos(Z) + np.sin(X)*np.sin(Z), x],
        #                       [np.cos(Y)*np.sin(Z), np.sin(X)*np.sin(Y)*np.sin(Z) + np.cos(X)*np.cos(Z), np.cos(X)*np.sin(Y)*np.sin(Z) - np.sin(X)*np.cos(Z), y],
        #                       [-np.sin(Y), np.sin(X)*np.cos(Y), np.cos(X)*np.cos(Y), z],
        #                       [0, 0, 0, 1]])

        #     ais.append(cam_t)
        #     transformations.append(np.dot(transformations[-1], cam_t))
        
        # Calculate feature jacobian
        z = 0.5
        f = (1024/2)/np.tan((1.5471/2))
        corner_feature_jacobians = []
        for i, corner in enumerate(self.corners):
            # corner_feature_jacobian = np.array([[-f/z, 0, self.corners[i][0]/z, self.corners[i][0]*self.corners[i][1]/f, -(f+self.corners[i][0]*self.corners[i][0]/f), self.corners[i][1]],
            #                                     [0, -f/z, self.corners[i][1]/z, f+self.corners[i][1]*self.corners[i][1]/f, -self.corners[i][0]*self.corners[i][1]/f, -self.corners[i][0]]])

            xc = self.corners[i][0]
            yc = self.corners[i][1]            
            corner_feature_jacobian = np.array([[-f/z, 0, xc/z, xc*yc/f, -(f+xc*xc/f), yc],
                                                [0, -f/z, yc/z, f+yc*yc/f, -xc*yc/f, -xc]])   # Correct Implementation
            
            corner_feature_jacobians.append(corner_feature_jacobian)
        image_jacobian = np.vstack(corner_feature_jacobians)

        # z = 0.5
        # f = (1024/2)/np.tan((1.5471/2))  # Correct after reference

        # xc = self.corners[0][0]
        # yc = self.corners[0][1]
        # image_jacobian = np.array([[-f/z, 0, xc/z, xc*yc/f, -(f+xc*xc/f), yc],
        #                            [0, -f/z, yc/z, f+yc*yc/f, -xc*yc/f, -xc]])   # Correct Implementation

        # Calculate robot jacobian using DH paremeters
        # robot_jacobian = None
        # end_effector_n = len(dh_parameters)
        # for i in range(len(dh_parameters)):
        #     if i == 0:
        #         joint_av = np.array([[0],[0],[1]])
        #         joint_lv = np.cross(np.array([[0],[0],[1]]), np.array([[transformations[end_effector_n - 1][0][3]],
        #                             [transformations[end_effector_n - 1][1][3]], [transformations[end_effector_n - 1][2][3]]]), axis=0)
                
        #         joint_jacobain = np.vstack((joint_lv,joint_av))
        #         robot_jacobian = joint_jacobain

        #     else:                                
        #         joint_av = np.array([[transformations[i - 1][0][2]], [transformations[i - 1][1][2]], [transformations[i - 1][2][2]]])
        #         joint_lv = np.cross(np.array([[transformations[i - 1][0][3]],[transformations[i - 1][1][3]], [transformations[i - 1][2][3]]]), 
        #                             np.array([[transformations[end_effector_n - 1][0][3]], [transformations[end_effector_n - 1][1][3]],
        #                                       [transformations[end_effector_n - 1][2][3]]]) - np.array([[transformations[i - 1][0][3]], [transformations[i - 1][1][3]],
        #                                       [transformations[i - 1][2][3]]]), axis=0)

        #         joint_jacobain = np.vstack((joint_lv,joint_av))
        #         robot_jacobian = np.hstack((robot_jacobian, joint_jacobain))

        # Calculate jacobian using screw axis
        # Defining link lengths to be used in screw axes and home configuration
        L1 = 0.333
        L2 = 0.316
        L3 = 0.0825
        L4 = 0.384
        L5 = 0.088 
        L6 = 0.107

        # Calculate the forward kinematics using the Product of Exponentials
        # Let us calculate the screw axis for each joint
        # Put all the axes into a 6xn matrix S, where n is the number of joints
        screw_axis = np.array([[0, 0,  1, 0,          0,  0],
                               [0, 1,  0, -L1,        0,  0],
                               [0, 0,  1, 0,          0,  0],
                               [0, -1, 0, (L1+L2),    0,  -L3],
                               [0, 0,  1, 0,          0,  0],
                               [0, -1, 0, (L1+L2+L4), 0,  0],
                               [0, 0, -1, 0,          L5, 0]])

        robot_jacobian = JacobianBody(screw_axis.T, np.array(self.joint_angles))
        # robot_jacobian = JacobianSpace(screw_axis.T, np.array(self.joint_angles))

        # Calculate jacobian using moveit API
        # robot_jacobian = self.move_group.get_jacobian_matrix(list(self.joint_angles))

        # Control law
        lam = -0.01
        lam_mat = np.eye(6)*lam

        # error = np.array([[self.corners[0][0] - self.reference_corners[0][0]],
        #                   [self.corners[0][1] - self.reference_corners[0][1]],
        #                   [self.corners[1][0] - self.reference_corners[1][0]],
        #                   [self.corners[1][1] - self.reference_corners[1][1]],
        #                   [self.corners[2][0] - self.reference_corners[2][0]],
        #                   [self.corners[2][1] - self.reference_corners[2][1]],
        #                   [self.corners[3][0] - self.reference_corners[3][0]],
        #                   [self.corners[3][1] - self.reference_corners[3][1]]])

        error = np.array([[self.corners[0][0] - 580],
                          [self.corners[0][1] - 580],
                          [self.corners[1][0] - 580],
                          [self.corners[1][1] - 620],
                          [self.corners[2][0] - 620],
                          [self.corners[2][1] - 620],
                          [self.corners[3][0] - 620],
                          [self.corners[3][1] - 580]])

        # error = np.array([[self.corners[0][0] - 500.0],
        #                   [self.corners[0][1] - 500.0]])

        # matrix sizes:
        # error: 8x1
        # image_jacobian: 8x6
        # reference_cartesian_velocities: 6x1  
        # robot_jacobian: 6x7
        # input_joint_velocities: 7x1   

        # reference_cartesian_velocities = np.dot(lam_mat, np.dot(np.linalg.pinv(image_jacobian), error))
        reference_cartesian_velocities = lam*np.dot(np.linalg.pinv(image_jacobian), error)

        # reference cartesian velocity based on screw axis jacobian
        # reference_cartesian_velocities = [reference_cartesian_velocities[1], reference_cartesian_velocities[0], reference_cartesian_velocities[2], reference_cartesian_velocities[3], reference_cartesian_velocities[4], reference_cartesian_velocities[5]]
        print(reference_cartesian_velocities)

        # reference_cartesian_velocities = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T
        # reference_cartesian_velocities = [0, 0, 0, -reference_cartesian_velocities[2], reference_cartesian_velocities[0], -reference_cartesian_velocities[1]]
        reference_cartesian_velocities = [0, 0, 0, reference_cartesian_velocities[0], reference_cartesian_velocities[1], reference_cartesian_velocities[2]]

        # reference_cartesian_velocities = [reference_cartesian_velocities[0], reference_cartesian_velocities[1], 0, 0, 0, 0]

        input_joint_velocities = np.dot(np.linalg.pinv(robot_jacobian), reference_cartesian_velocities)

        self.pub_q1_vel.publish(input_joint_velocities[0])
        self.pub_q2_vel.publish(input_joint_velocities[1])
        self.pub_q3_vel.publish(input_joint_velocities[2])
        self.pub_q4_vel.publish(input_joint_velocities[3])
        self.pub_q5_vel.publish(input_joint_velocities[4])
        self.pub_q6_vel.publish(input_joint_velocities[5])
        self.pub_q7_vel.publish(input_joint_velocities[6])
        print("------------------")
        print(error)
        # print(input_joint_velocities)

if __name__ == "__main__":
    vs = VisualServoing()
    rospy.spin()