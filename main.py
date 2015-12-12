#!/usr/bin/env python
import cv2 as cv
import lib.visionlib as vl
import lib.motionlib as ml
from lib.game import Game 
import rospy
import numpy as np

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from bigredrobot_final.msg import *

from std_msgs.msg import (
    UInt16,
    Header,
    String
)

from geometry_msgs.msg import Vector3Stamped, Vector3

import tf

USE_TRACKBARS = False
#DEPTH_K0 = 0.003
DEPTH_K0 = 0.000005
XY_K0 = 0.0008
DEFENSE_K0 = 0.8

BOARD_MAX_X = 1.0
BOARD_MIN_X = 0.1

class Baxter:
    def __init__(self, arm):
        rospy.logwarn("Initializing Baxter")

        self.arm = baxter_interface.limb.Limb(arm)
                
        self.gripper = baxter_interface.Gripper(arm)
        self.gripper.calibrate()
        self.kinematics = baxter_kinematics(arm)

        self.interface = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self.interface.state().enabled
        self.interface.enable()
        
        self.gripper.open()


    def grip_ball(self):
        self.gripper.close()
        
class Planner:

    DEFENSE_MODE = 0
    ATTACK_MODE = 1

    BALL_Z = -0.077#-0.07837517

    ATTACK_POS = {'right_s0': -0.4893398707763672, 'right_s1': -0.03298058690185547, 'right_w0': -1.6582332298095703, 'right_w1': 1.4089613520629884, 'right_w2': -1.2168302585998536, 'right_e0': 1.440024462982178, 'right_e1': 1.6118303110290528}
    
    DEFENSE_POS = {'right_s0': -0.7378447581298828, 'right_s1': 0.2573252768737793, 'right_w0': -1.9105730691284182, 'right_w1': 1.2954467738891602, 'right_w2': -1.3115535721435547, 'right_e0': 1.3337962935424805, 'right_e1': 1.6463448787170412}

    # max and min correspond to goal bounds as percentages of the board width
    MAX_BOARD_WIDTH = 0.73
    MIN_BOARD_WIDTH = 0.27
    
    def __init__(self, game, camera, motion_controller):
        self.camera = camera
        #self.overhead_camera = overhead_camera
        self.motion_controller = motion_controller
        self.current_mode = Planner.DEFENSE_MODE
        self.tl = tf.TransformListener()
        rospy.Subscriber("overhead_camera", OverheadCamera, self.overheadCameraCallback)

    def overheadCameraCallback(self, data):
        self.ball_x = data.ball_x
        self.ball_y = data.ball_y
        # Get the current position of the ball relative to the width of the board
        # This thing is a percentage of the width
        self.ball_rel_pos = ((float(data.center_y)+float(data.height/2)) - data.ball_y)/float(data.height) 
        #print self.ball_rel_pos

    def run(self):
        arm = self.motion_controller._arm_obj
        
        window_name = 'Wrist Camera'
        cv.namedWindow(window_name)
        
        ## Check phase
        #phase = self.game.get_current_phase()
        phase = Game.PHASE_III
        if phase == Game.PHASE_I or phase == Game.NOT_RUNNING:
            # nothing happens (wait for phase 2)
            pass 
            
        elif phase == Game.PHASE_II:     
            ## Stack blocks
    
            stack_pub = rospy.Publisher('command', String)
            stack_pub.publish("stack_ascending")
            
            # # Maybe check time remaining? Otherwise just go until
            #remaining_time = game.get_remaining_time() # duration
            #state_update_time = game.get_update_time() # time
            #while rospy.Time.now()
            #    pass
                
        elif phase == Game.PHASE_III:
            # Stop stacking blocks    
            #stack_pub.publish("stop")
        
            # Start offense/defenSe
            self.enter_defense_mode()
            #self.enter_attack_mode()
            print 'Press <esc> to toggle modes'
            while not rospy.is_shutdown():
                self.raw = cv.blur(camera.get_frame(), (3,3)) 
                
                cv.imshow(window_name, self.raw)
                self.frame = cv.cvtColor(self.raw,cv.COLOR_BGR2HSV)
                #self.overhead_frame = overhead_camera.get_frame()
                k = cv.waitKey(5)
                if self.current_mode == Planner.DEFENSE_MODE:
                    self.defend()
                    if k == 27:
                        self.enter_attack_mode()
                else:
                    self.attack(camera)
                    if k == 27:
                        self.enter_defense_mode()                
    
    def defend(self):
        #pass
        #raw_input('Press Enter to see joint angles...')
        #print self.motion_controller._arm_obj.joint_angles()
        #raw_input('Press Enter to see gripper z coordinate...')
        #print self.motion_controller.get_gripper_coords()[2]
        self.defense_visual_servo()

    def enter_attack_mode(self):
        print 'Entering Attack Mode'
        self.current_mode = Planner.ATTACK_MODE
        self.motion_controller.set_joint_positions(Planner.ATTACK_POS)

    def enter_defense_mode(self):
        print 'Entering Defense Mode'
        self.current_mode = Planner.DEFENSE_MODE
        self.motion_controller.set_joint_positions(Planner.DEFENSE_POS)


    def attack_visual_servo(self):
        u, v, mask2, current_area = vl.locate_pink_ball(self.frame)
        #u, v, mask2, current_area = vl.locate_orange_ball(self.frame)
        #print current_area
        current_z = self.motion_controller.get_gripper_coords()[2][0]
        cv.imshow('MASK',mask2)
        # No object is being tracked
        if u == -1 or v == -1:
            xi = np.zeros(2) # xy vel to zero
            desired_depth_vel = 0.0
        else:
            xi = camera.calc_desired_feat_velocities(u, v, XY_K0)
            desired_depth_vel = camera.calc_desired_depth_velocity_z(DEPTH_K0, current_area)

        if current_z <= Planner.BALL_Z:
            baxter.grip_ball()
            rospy.sleep(0.2)
            print "I found the ball!"
            motion_controller.command_velocity(np.zeros(6))
            return True
        #print 'Desired depth vel:', desired_depth_vel
        xi = -np.append(xi, desired_depth_vel)
        #print xi
        vect = Vector3Stamped()
        vect.header.frame_id = '/right_hand_camera'
        vect.header.stamp = rospy.Time(0)
        vect.vector = Vector3(*xi[0:3])
        try:
            trans_vect = self.tl.transformVector3('/base', vect)
            squiggle = np.array([trans_vect.vector.x,trans_vect.vector.y,trans_vect.vector.z,0,0,0])
            #print squiggle
            motion_controller.command_velocity(squiggle)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "you suck"
        
        return False

    def defense_visual_servo(self):
        current_ball_rel_pos = self.ball_rel_pos
        if current_ball_rel_pos < 0:
            #If we cannot see the ball, stop movement
            motion_controller.command_velocity(np.zeros(6))
            return False
        # These two lines limit ball_rel_pos to be between the goal posts
        current_ball_rel_pos = min(Planner.MAX_BOARD_WIDTH, current_ball_rel_pos)
        current_ball_rel_pos = max(Planner.MIN_BOARD_WIDTH, current_ball_rel_pos)

        ball_pos = current_ball_rel_pos*(BOARD_MAX_X - BOARD_MIN_X) + BOARD_MIN_X
        hand_pos = self.motion_controller.get_gripper_coords()[0]

        x_vel = DEFENSE_K0*(ball_pos - hand_pos)
        print 'x_vel', x_vel
        squiggle = np.array([x_vel,0,0,0,0,0])

        try:
            motion_controller.command_velocity(squiggle)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "you suck"
        
        return False

    def attack(self, camera):
        if self.get_ball(camera):
            self.motion_controller.throw()
            self.enter_defense_mode()
            
        
    #TODO: Clean up this huge function
    def get_ball(self, camera):
       
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

        #orng_x, orng_y, mask1, current_area = vl.locate_orange_ball(frame)
        pink_x, pink_y, mask2, current_area = vl.locate_pink_ball(frame)

        display = frame.copy()
        cv.circle(display, (320, 70), 10, 255, -1) 
        #cv.circle(display, (orng_x, orng_y), 10, (0,0,255), -1) 
        cv.circle(display, (pink_x, pink_y), 10, (0,255,0), -1) 

        #cv.imshow('Thresh orange',mask1)
        #cv.imshow('Thresh pink',mask2)
        #cv.imshow('result', display)
        
        #ball_gripped = self.visual_servo(self.tl, camera, pink_x, pink_y, current_area)
        ball_gripped = self.attack_visual_servo()
        # rate.sleep()
        
        if ball_gripped:
            print 'THROW'
            return True
        else:
            return False

        #k = cv.waitKey(5) & 0xFF
        #if k == 27:
        #    break
        
        #cv.destroyAllWindows()
def debug_with_trackbars():

    cv.namedWindow('result')
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
        hmin = cv.getTrackbarPos('hmin','result')
        hmax = cv.getTrackbarPos('hmax','result')
        smin = cv.getTrackbarPos('smin','result')
        smax = cv.getTrackbarPos('smax','result')
        vmin = cv.getTrackbarPos('vmin','result')
        vmax = cv.getTrackbarPos('vmax','result')
        lower_hsv = np.array([hmin, smin, vmin])
        upper_hsv = np.array([hmax, smax, vmax])
        mask_trackbar = cv.inRange(hsv,lower_hsv, upper_hsv)
        track_x, track_y, mask3, area = vl.track_object(mask_trackbar)
        #print area
        cv.imshow('Thresh trackbar',mask3)

        display = frame.copy()
        cv.circle(display, (track_x, track_y), 10, (0,0,255), -1) 

        cv.imshow('result', display)

        rate.sleep()

        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
        
    cv.destroyAllWindows()
    
    
    

if __name__=="__main__":
    rospy.init_node('main', anonymous=True)

    game = Game() # start communication with game server (get arm etc.)
    
    #baxter = Baxter(arm='right') # remove for the competition
    baxter = Baxter(arm=game.get_arm())  
    motion_controller = ml.BaxterMotionController(baxter, arm='right')
    camera = vl.BaxterCamera(arm='right')
    planner = Planner(game, camera, motion_controller)
    #debug_with_trackbars()
    planner.run()

