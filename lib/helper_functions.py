#!/usr/bin/env python

import numpy as np
import rospy
import rospkg
import xml.etree.ElementTree as ET
from scipy import interpolate

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

def get_plane(p1, p2, p3):
    '''
    Returns a unit vector normal to the 3 input points    
    '''
    r12 = p2 - p1
    r32 = p2 - p3
    normal = np.matrix(np.cross(r12.A1, r32.A1)).T
    normal = normal/np.linalg.norm(normal)
    return p1, normal

def project_point(point, normal, q):
    '''
    Projects q into the plane defined by normal, point
    '''
    return q - np.dot(q.A1 - point.A1, normal.A1) * normal

def save_plane(point, normal):
    '''
    Saves plane points to file
    '''
    rospack = rospkg.RosPack()
    path = rospack.get_path("bigredrobot_proj2")
    f = open(path+PLANE_FILE, 'w')
    f.write(str(point.item(0)) + ":" + str(point.item(1)) + ":" + str(point.item(2)) + '\n')
    f.write(str(normal.item(0)) + ":" + str(normal.item(1)) + ":" + str(normal.item(2)) + '\n')
    f.close()

def save_goal(start, goal):
    '''
    Saves plane points to file
    '''
    rospack = rospkg.RosPack()
    path = rospack.get_path("bigredrobot_proj2")
    f = open(path+GOAL_FILE, 'w')
    f.write(str(start.item(0)) + ":" + str(start.item(1)) + ":" + str(start.item(2)) + '\n')
    f.write(str(goal.item(0)) + ":" + str(goal.item(1)) + ":" + str(goal.item(2)) + '\n')
    f.close()


def load_plane():
    '''
    Reads points from plane file and returns the plane
    '''
    rospack = rospkg.RosPack()
    path = rospack.get_path("bigredrobot_proj2")
    f = open(path+PLANE_FILE, 'r')

    #first line is point
    line = f.readline()
    coords = [x.strip() for x in line.split(':')]
    point = np.matrix([float(coords[0]),float(coords[1]),float(coords[2])]).T


    #second line is norm
    line = f.readline()
    coords = [x.strip() for x in line.split(':')]
    norm = np.matrix([float(coords[0]),float(coords[1]),float(coords[2])]).T

    f.close()

    return point, norm

def load_goal():
    '''
    Reads points from plane file and returns the plane
    '''
    rospack = rospkg.RosPack()
    path = rospack.get_path("bigredrobot_proj2")
    f = open(path+GOAL_FILE, 'r')

    #first line is point
    line = f.readline()
    coords = [x.strip() for x in line.split(':')]
    start = np.matrix([float(coords[0]),float(coords[1]),float(coords[2])]).T


    #second line is norm
    line = f.readline()
    coords = [x.strip() for x in line.split(':')]
    goal = np.matrix([float(coords[0]),float(coords[1]),float(coords[2])]).T

    f.close()

    return start, goal

def get_spline(p1, p2, p3, p4):
    '''
    Interpolates between 4 points using a cubic spline, returns spline
    representation and parameter values 'u'
    '''
    x = np.array([p1[0,0], p2[0,0], p3[0,0], p4[0,0]])
    y = np.array([p1[1,0], p2[1,0], p3[1,0], p4[1,0]])
    z = np.array([p1[2,0], p2[2,0], p3[2,0], p4[2,0]])
    rospy.logwarn(x)
    tck, u = interpolate.splprep([x, y, z])
    print 'u:', u
    return tck, u


if __name__ == '__main__':
    load_plane()
