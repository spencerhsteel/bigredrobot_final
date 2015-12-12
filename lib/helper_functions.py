#!/usr/bin/env python

import numpy as np
import rospy
import rospkg
import xml.etree.ElementTree as ET

frame_dict_right = { "right_s0": "right_upper_shoulder",
                    "right_s1": "right_lower_shoulder",
                    "right_e0": "right_upper_elbow",
                    "right_e1": "right_lower_elbow",
                    "right_w0": "right_upper_forearm",
                    "right_w1": "right_lower_forearm",
                    "right_w2": "right_wrist"}
                    
frame_dict_left = { "left_s0": "left_upper_shoulder",
                    "left_s1": "left_lower_shoulder",
                    "left_e0": "left_upper_elbow",
                    "left_e1": "left_lower_elbow",
                    "left_w0": "left_upper_forearm",
                    "left_w1": "left_lower_forearm",
                    "left_w2": "left_wrist"}


PLANE_FILE = '/scripts/plane.txt'
GOAL_FILE = '/scripts/goal.txt'

def get_frame_dict(arm):
    return frame_dict_right if arm == 'right' else frame_dict_left

def get_limits(arm):
    '''
    Returns two dictionaries of lower and upper joint limits for all joints in the given arm
    '''
    
    frame_dict = get_frame_dict(arm)
    
    rospack = rospkg.RosPack()
    path = rospack.get_path("baxter_description")
    tree = ET.parse(path + "/urdf/baxter.urdf")
    root = tree.getroot()

    lower_limits = {}
    upper_limits = {}

    for joint_name in frame_dict.keys():
        joint_element = root.find(".//joint[@name='%s']" %(joint_name))
        limit_element = joint_element.find('limit')        
        lower_limits[joint_name] = float(limit_element.get('lower'))
        upper_limits[joint_name] = float(limit_element.get('upper'))
    
    return lower_limits, upper_limits

def rpinv(J):
    '''
    Right pseudo inverse
    '''
    return np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))



if __name__ == '__main__':
    load_plane()
