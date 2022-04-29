#!/usr/bin/env python

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
from cvxopt import matrix, solvers

# cv2.namedWindow('draw frame',cv2.WND_PROP_FULLSCREEN)
# cv2.setWindowProperty('draw frame', cv2.WND_PROP_FULLSCREEN, cv2.WND_PROP_FULLSCREEN)

def find_center(img, color):
    ##########
    # The algorithm from homework 3
    # Convert the image to hsv
    # Find the range of the color in hsv color space
    # Segment the required colors using the range
    # Find the center of the segmented objects
    ##########

    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    
    if color == 'red':
        low_thresh = np.array([75, 40, 40])
        high_thresh = np.array([122, 255, 255])
    elif color == 'green':
        low_thresh = np.array([50, 50, 50])
        high_thresh = np.array([100, 255, 255])
    elif color == 'blue':
        low_thresh = np.array([0, 50, 50])
        high_thresh = np.array([50, 255, 255])
    elif color == 'pink':
        low_thresh = np.array([150, 50, 50])
        high_thresh = np.array([200, 255, 255])

    mask = cv2.inRange(img_hsv, low_thresh, high_thresh)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]

    cv2.drawContours(img_hsv, contours, 0, (0, 0, 0), 2)

    M = cv2.moments(contours[0])

    center = [int(round(M['m10'] / M['m00'])), int(round(M['m01'] / M['m00']))]
    # rospy.loginfo('The center of %s object %s, %s', color, center[0], center[1])
    
    # cv2.circle(img_hsv, (center[0], center[1]), 5, (255, 255, 255), -1)

    img_rgb = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2RGB)
    return center, img_rgb

def joint_state_cb(data):
    global joint_angles
    joint_angles = data.position

def detect_image_cb(data):
    ##########
    # Receive image from rostopic
    # Convert to opencv datatype
    ##########
    global joint_angles, green_x_pub, green_y_pub, pink_x_pub, pink_y_pub, red_x_pub, red_y_pub, blue_x_pub, blue_y_pub, init_time, pub_q1_vel, pub_q2_vel, image_pub, frame_count, prev_coords
    frame_count = frame_count + 1
    curr_time = rospy.Time.now()
    time_diff = curr_time-init_time
    
    img = bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
    visualize = img.copy()

    red_center, red_obj = find_center(img, 'red')
    blue_center, blue_obj = find_center(img, 'blue')
    green_center, green_obj = find_center(img, 'green')
    pink_center, pink_obj = find_center(img, 'pink')
        
    # Displays the output of object segmentation
    # cv2.imshow('red_object', red_obj)
    # cv2.imshow('blue_object', blue_obj)
    # cv2.imshow('green_object', green_obj)
    # cv2.imshow('pink_object', pink_obj)
    # cv2.imshow('original frame', img)
    # key = cv2.waitKey(1)
    
    # feature references 1 - [green: (63, 187), pink: (121, 218), red: (107, 174), blue: (77, 231)], joint ref - [joint1 0.2, joint2 0.1]
    # feature references 3 - [green: (191, 173), pink: (221, 231), red: (235, 187), blue: (178, 217)] - [joint1 -0.2, joint2 -0.1]
    # feature references 2 - [green: (126, 169), pink: (172, 215), red: (173, 169), blue: (127, 215)] - [joint1 0.0, joint2 0.0]
    
    # reference_features = [[63, 187], [121, 218], [107, 174], [77, 231]]      #[green, pink, red, blue]
    # reference_features = [[191, 173], [221, 231], [235, 187], [178, 217]]      #[green, pink, red, blue]
    # reference_features = [[126, 169], [172, 215], [173, 169], [127, 215]]      #[green, pink, red, blue]

    # reference_features = [[226, 126], [277, 158], [267, 117], [236, 168]]      #[green, pink, red, blue]
    reference_features = [[215, 83], [265, 114], [257, 73], [225, 124]]      #[green, pink, red, blue]
    
    visualize = cv2.circle(visualize, tuple(reference_features[0]), 4, (0, 0, 0), -1)
    visualize = cv2.circle(visualize, tuple(reference_features[1]), 4, (0, 0, 0), -1)
    visualize = cv2.circle(visualize, tuple(reference_features[2]), 4, (0, 0, 0), -1)
    visualize = cv2.circle(visualize, tuple(reference_features[3]), 4, (0, 0, 0), -1)

    visualize = cv2.circle(visualize, tuple(red_center), 4, (255, 255, 255), -1)
    visualize = cv2.circle(visualize, tuple(blue_center), 4, (255, 255, 255), -1)
    visualize = cv2.circle(visualize, tuple(green_center), 4, (255, 255, 255), -1)
    visualize = cv2.circle(visualize, tuple(pink_center), 4, (255, 255, 255), -1)

    if frame_count > 1:
        for i in range(1, len(prev_coords[0])):
            visualize = cv2.line(visualize, tuple(prev_coords[0][i - 1]), tuple(prev_coords[0][i]), (0,191,255), 2)
            visualize = cv2.line(visualize, tuple(prev_coords[1][i - 1]), tuple(prev_coords[1][i]), (0,191,255), 2)
            visualize = cv2.line(visualize, tuple(prev_coords[2][i - 1]), tuple(prev_coords[2][i]), (0,191,255), 2)
            visualize = cv2.line(visualize, tuple(prev_coords[3][i - 1]), tuple(prev_coords[3][i]), (0,191,255), 2)

    prev_coords[0].append(list(green_center))
    prev_coords[1].append(list(pink_center)) 
    prev_coords[2].append(list(red_center)) 
    prev_coords[3].append(list(blue_center)) 

    image_pub.publish(bridge.cv2_to_imgmsg(visualize, encoding='rgb8'))

    l1 = 0.5 - 0.05
    l2 = 0.5 - 0.05
        
    a1 = np.array([[np.cos(joint_angles[0]), -np.sin(joint_angles[0]), 0, l1*np.cos(joint_angles[0])],
                   [np.sin(joint_angles[0]),  np.cos(joint_angles[0]), 0, l1*np.sin(joint_angles[0])],
                   [0                      ,                       0,  1,                          0],
                   [0                      ,                       0,  0,                          1]])
    a2 = np.array([[np.cos(joint_angles[1]), -np.sin(joint_angles[1]), 0, l2*np.cos(joint_angles[1])],
                   [np.sin(joint_angles[1]),  np.cos(joint_angles[1]), 0, l2*np.sin(joint_angles[1])],
                   [0                      ,                       0,  1,                          0],
                   [0                      ,                       0,  0,                          1]]) 
    
    t0_1 = a1
    t0_2 = np.matmul(a1, a2)
    f = 1
    z = 1
    
    green_feature_jacobian = np.array([[-1/z, 0, green_center[0]/z, green_center[0]*green_center[1], -(1+green_center[0]*green_center[0]), green_center[1]],
                                       [0, -1/z, green_center[1]/z, 1+green_center[1]*green_center[1], -green_center[0]*green_center[1], -green_center[0]]])
    
    pink_feature_jacobian = np.array([[-1/z, 0, pink_center[0]/z, pink_center[0]*pink_center[1], -(1+pink_center[0]*pink_center[0]), pink_center[1]],
                                       [0, -1/z, pink_center[1]/z, 1+pink_center[1]*pink_center[1], -pink_center[0]*pink_center[1], -pink_center[0]]])
   
    red_feature_jacobian = np.array([[-1/z, 0, red_center[0]/z, red_center[0]*red_center[1], -(1+red_center[0]*red_center[0]), red_center[1]],
                                       [0, -1/z, red_center[1]/z, 1+red_center[1]*red_center[1], -red_center[0]*red_center[1], -red_center[0]]])
   
    blue_feature_jacobian = np.array([[-1/z, 0, blue_center[0]/z, blue_center[0]*blue_center[1], -(1+blue_center[0]*blue_center[0]), blue_center[1]],
                                       [0, -1/z, blue_center[1]/z, 1+blue_center[1]*blue_center[1], -blue_center[0]*blue_center[1], -blue_center[0]]])
     
    image_jacobian = np.array(np.vstack((green_feature_jacobian, pink_feature_jacobian, red_feature_jacobian, blue_feature_jacobian)))
      
    joint1_lv = np.cross(np.array([[0],[0],[1]]), np.array([[t0_2[0][3]],[t0_2[1][3]],[t0_2[2][3]]]), axis=0)
    joint1_av = np.array([[0],[0],[1]])
    joint2_lv = np.cross(np.array([[t0_1[0][3]],[t0_1[1][3]],[t0_1[2][3]]]), \
                np.array([[t0_2[0][3]],[t0_2[1][3]],[t0_2[2][3]]]) - np.array([[t0_1[0][3]],[t0_1[1][3]],[t0_1[2][3]]]), axis=0)
    joint2_av = np.array([[t0_1[0][3]],[t0_1[1][3]],[t0_1[2][3]]])
    
    robot_jacobian = np.vstack((np.hstack((joint1_lv, joint2_lv)), np.hstack((joint1_av, joint2_av))))
    
    lam = 0.01
    lam_mat = np.eye(6)*lam
        
    error = np.array([[green_center[0] - reference_features[0][0]],
                      [green_center[1] - reference_features[0][1]],
                       [pink_center[0] - reference_features[1][0]],
                       [pink_center[1] - reference_features[1][1]],
                       [red_center[0] - reference_features[2][0]],
                       [red_center[1] - reference_features[2][1]],
                       [blue_center[0] - reference_features[3][0]],
                       [blue_center[1] - reference_features[3][1]]])


    reference_cartesian_velocities = np.matmul(lam_mat, np.matmul(np.linalg.pinv(image_jacobian), error))
    input_joint_velocities = -np.matmul(np.linalg.pinv(robot_jacobian), reference_cartesian_velocities)

    if(rospy.Duration(5) < time_diff):
        rospy.loginfo('controller started')
        # pub_q1_vel.publish(input_joint_velocities[0])
        # pub_q2_vel.publish(input_joint_velocities[1])


    green_x_pub.publish(green_center[0])
    green_y_pub.publish(green_center[1])
    pink_x_pub.publish(pink_center[0])
    pink_y_pub.publish(pink_center[1])
    red_x_pub.publish(red_center[0])
    red_y_pub.publish(red_center[1])
    blue_x_pub.publish(blue_center[0])
    blue_y_pub.publish(blue_center[1])
    
    print(red_center, blue_center, green_center, pink_center)

if __name__ == "__main__":
    rospy.init_node('detect_features_node')

    rospy.sleep(1)
    init_time = rospy.Time.now()
    rospy.loginfo('init time %s', rospy.Time.now())
    
    bridge = CvBridge()
    frame_count = 0
    prev_coords = [[], [], [], []]
    joint_angles = np.array([0, 0])
    green_x_pub = rospy.Publisher('/vbmbot/green_x', Float64 ,queue_size=10)
    green_y_pub = rospy.Publisher('/vbmbot/green_y', Float64 ,queue_size=10)
    pink_x_pub = rospy.Publisher('/vbmbot/pink_x', Float64 ,queue_size=10)
    pink_y_pub = rospy.Publisher('/vbmbot/pink_y', Float64 ,queue_size=10)
    red_x_pub = rospy.Publisher('/vbmbot/red_x', Float64 ,queue_size=10)
    red_y_pub = rospy.Publisher('/vbmbot/red_y', Float64 ,queue_size=10)
    blue_x_pub = rospy.Publisher('/vbmbot/blue_x', Float64 ,queue_size=10)
    blue_y_pub = rospy.Publisher('/vbmbot/blue_y', Float64 ,queue_size=10)

    # Visual servoing
    pub_q1_vel = rospy.Publisher('/vbmbot/joint1_velocity_controller/command', Float64, queue_size=10)
    pub_q2_vel = rospy.Publisher('/vbmbot/joint2_velocity_controller/command', Float64, queue_size=10)

    image_pub = rospy.Publisher('/visualization', Image, queue_size=10)

    rospy.Subscriber('/vbmbot/camera1/image_raw', Image, detect_image_cb)
    rospy.Subscriber('/vbmbot/joint_states', JointState, joint_state_cb)

    rospy.spin()

