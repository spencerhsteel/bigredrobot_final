import cv2 as cv
import lib.visionlib as vl
import lib.motionlib as ml
import rospy
import numpy as np

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

from std_msgs.msg import (
    UInt16,
    Header
)

from geometry_msgs.msg import Vector3Stamped, Vector3

import tf

USE_TRACKBARS = False
DEPTH_K0 = 0.002
XY_K0 = 0.0005

class Baxter:
    def __init__(self, arm):
        rospy.logwarn("Initializing Baxter")

        self.arm = baxter_interface.limb.Limb(arm)
        self.gripper = baxter_interface.Gripper(arm)
        self.gripper.calibrate()
        self.kinematics = baxter_kinematics(arm)

        # control parameters
        self.pub_rate = 500.0  # Hz

        self.interface = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self.interface.state().enabled
        self.interface.enable()
        
        # set joint state publishing to 500Hz
        self.rate_publisher = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self.rate_publisher.publish(self.pub_rate)
        self.gripper.close()
        self.gripper.open()

    def grip_ball(self):
        self.gripper.close()
        
class Planner:
    DEFENSE_MODE = 0
    OFFENSE_MODE = 1
    
    ATTACK_POS = {'right_s0': -0.1729563336364746, 'right_s1': -0.9407137170959473, 'right_w0': -0.2538738201049805, 'right_w1': 0.783480686517334, 'right_w2': -2.023704152105713, 'right_e0': 0.411490345880127, 'right_e1': 1.7353157643127441}
    
    DEFENSE_POS = {'right_s0': -0.7378447581298828, 'right_s1': 0.2573252768737793, 'right_w0': -1.9105730691284182, 'right_w1': 1.2954467738891602, 'right_w2': -1.3115535721435547, 'right_e0': 1.3337962935424805, 'right_e1': 1.6463448787170412}

    
    def __init__(self, camera, motion_controller):
        self.camera = camera
        #self.overhead_camera = overhead_camera
        self.motion_controller = motion_controller
        self.current_mode = Planner.DEFENSE_MODE
        self.tl = tf.TransformListener()
        
        window_name = 'Wrist Camera'
        cv.namedWindow(window_name)
        cv.namedWindow('mask')
        
        arm = self.motion_controller._arm_obj
        
        self.enter_defense_mode(arm)
        
        while not rospy.is_shutdown():
            self.frame = camera.get_frame()
            #self.overhead_frame = overhead_camera.get_frame()
            cv.imshow(window_name, self.frame)
            if self.current_mode == Planner.DEFENSE_MODE:
                self.defend()
                #pass
            else:
                self.attack(camera)
            cv.waitKey(5)
    
    def defend(self):
        raw_input('Press Enter to see joint angles...')
        print self.motion_controller._arm_obj.joint_angles()
        #self.defense_visual_servo()

    def enter_defense_mode(self, arm):
        print 'Entering Defense Mode'
        arm.move_to_joint_positions(Planner.DEFENSE_POS)
        #arm.set_joint_positions(Planner.DEFENSE_POS) # run this when first entering defenCe mode

    def defense_visual_servo(self):
        u, v, mask2, current_area = vl.locate_pink_ball(self.overhead_frame)
        cv.imshow('mask', mask2)
        
        # No object is being tracked
        if u == -1 or v == -1:
            xi = np.zeros(2) # xy vel to zero
        else:
            print 'object found:', u, v
            xi = camera.calc_desired_feat_velocities(u, v, XY_K0)
            xi[1] = 0.0 # don't move in camera y
            
        xi = -np.append(xi, desired_depth_vel)
        print xi
        
    '''
    def defense_visual_servo_old(self):
        u, v, mask2, current_area = vl.locate_pink_ball(self.frame)
        #u, v, mask2, current_area = vl.locate_orange_ball(self.frame)
        desired_depth_vel = 0
        
        cv.imshow('mask',mask2)
        
        # No object is being tracked
        if u == -1 or v == -1:
            xi = np.zeros(2) # xy vel to zero
        else:
            print 'object found:', u, v
            xi = camera.calc_desired_feat_velocities(u, v, XY_K0)
            xi[1] = 0.0 # don't move in camera y
            
        xi = -np.append(xi, desired_depth_vel)
        print xi

        vect = Vector3Stamped()
        vect.header.frame_id = '/right_hand_camera'
        vect.header.stamp = rospy.Time(0)
        vect.vector = Vector3(*xi[0:3])

        try:
            trans_vect = self.tl.transformVector3('/base', vect)
            squiggle = np.array([trans_vect.vector.x,trans_vect.vector.y,trans_vect.vector.z,0,0,0])
            print squiggle
            motion_controller.command_velocity(squiggle)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "you suck"
        
        return False
    '''         
























    def attack(self, camera):
        self.get_ball(camera)
        
    #TODO: Clean up this huge function
    def get_ball(self, camera):
        tl = tf.TransformListener()

        # Creating track bar
        if USE_TRACKBARS:
            callback = lambda x: 0
            cv.createTrackbar('hmin', 'result', 0, 179, callback)
            cv.createTrackbar('hmax', 'result',179,179, callback)
            cv.createTrackbar('smin', 'result',0,255, callback)
            cv.createTrackbar('smax', 'result',255,255, callback)
            cv.createTrackbar('vmin', 'result',0,255, callback)
            cv.createTrackbar('vmax', 'result',255,255, callback)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            #_, self.frame = self.capture.read()
            frame = cv.blur(camera.get_frame(), (3,3))

            #converting to HSV
            hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)

            # get info from track bar and appy to result
            if USE_TRACKBARS:
                hmin = cv.getTrackbarPos('hmin','result')
                hmax = cv.getTrackbarPos('hmax','result')
                smin = cv.getTrackbarPos('smin','result')
                smax = cv.getTrackbarPos('smax','result')
                vmin = cv.getTrackbarPos('vmin','result')
                vmax = cv.getTrackbarPos('vmax','result')
                lower_hsv = np.array([hmin, smin, vmin])
                upper_hsv = np.array([hmax, smax, vmax])
                mask_trackbar = cv.inRange(hsv,lower_hsv, upper_hsv)
                track_x, track_y, mask3, _ = vl.track_object(mask_trackbar)
                cv.imshow('Thresh trackbar',mask3)


            orng_x, orng_y, mask1, current_area = vl.locate_orange_ball()
            pink_x, pink_y, mask2, current_area = vl.locate_pink_ball()


            display = frame.copy()
            cv.circle(display, (320, 70), 10, 255, -1) 
            cv.circle(display, (orng_x, orng_y), 10, (0,0,255), -1) 
            #cv.circle(display, (pink_x, pink_y), 10, (0,255,0), -1) 

            cv.imshow('Thresh orange',mask1)
            #cv.imshow('Thresh pink',mask2)
            #cv.imshow('result', display)
            
            #ball_gripped = visual_servo(tl, camera, pink_x, pink_y, current_area)
            ball_gripped = visual_servo(tl, camera, orng_x, orng_y, current_area)
            # rate.sleep()
            
            if ball_gripped:
                print 'THROW'
                break

            k = cv.waitKey(5) & 0xFF
            if k == 27:
                break
        
        cv.destroyAllWindows()

    # NOTE: tl is transform listener
    def visual_servo(self, tl, camera, u, v, current_area):    
        
        # No object is being tracked
        if u == -1 or v == -1:
            xi = np.zeros(2) # xy vel to zero
            desired_depth_vel = 0.02
        else:
            xi = camera.calc_desired_feat_velocities(u, v, XY_K0)
            desired_depth_vel = camera.calc_desired_depth_velocity(DEPTH_K0, current_area)
            if desired_depth_vel > 0:
                baxter.grip_ball()   
                print "I found the ball!" 
                motion_controller.command_velocity(np.zeros(6))
                return True
        
        print 'Desired depth vel:', desired_depth_vel

        xi = -np.append(xi, desired_depth_vel)
        print xi

        vect = Vector3Stamped()
        vect.header.frame_id = '/right_hand_camera'
        vect.header.stamp = rospy.Time(0)
        vect.vector = Vector3(*xi[0:3])

        try:
            trans_vect = tl.transformVector3('/base', vect)
            squiggle = np.array([trans_vect.vector.x,trans_vect.vector.y,trans_vect.vector.z,0,0,0])
            print squiggle
            motion_controller.command_velocity(squiggle)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "you suck"
        
        return False


if __name__=="__main__":
    rospy.init_node('main', anonymous=True)
    baxter = Baxter(arm='right')
    motion_controller = ml.BaxterMotionController(baxter, arm='right')
    #transform_test()
    camera = vl.BaxterCamera(arm='right')
    planner = Planner(camera, motion_controller)
