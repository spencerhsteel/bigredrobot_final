#!/usr/bin/env python
import cv2 as cv
import lib.visionlib as vl
import lib.motionlib as ml
from lib.game import Game 
import rospy, baxter_interface, tf
import numpy as np

from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from bigredrobot_final.msg import *

from std_msgs.msg import UInt16, Header, String
from geometry_msgs.msg import Vector3Stamped, Vector3

########## CALIBRATION SWITCH VAR
CALIBRATE = False
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

        self.arm_obj = baxter_interface.limb.Limb(arm)
                
        self.gripper = baxter_interface.Gripper(arm)
        #self.gripper.calibrate() ###################### GRIPPER CALIBRATE, DO WE NEED THIS?

        self.interface = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self.interface.state().enabled
        self.interface.enable()
        
        #self.gripper.open() ########################### GRIPPER OPEN, DO WE NEED THIS?

    def grip_ball(self):
        self.gripper.close()
        
        
class Planner:

    DEFENSE_MODE = 0
    ATTACK_MODE = 1

    BALL_Z = -0.077#-0.07837517

    ATTACK_POS_RIGHT_CONST = {'right_s0': -0.4893398707763672, 'right_s1': -0.03298058690185547, 'right_w0': -1.6582332298095703, 'right_w1': 1.4089613520629884, 'right_w2': -1.2168302585998536, 'right_e0': 1.440024462982178, 'right_e1': 1.6118303110290528}
    
    DEFENSE_POS_RIGHT_CONST = {'right_s0': -0.7378447581298828, 'right_s1': 0.2573252768737793, 'right_w0': -1.9105730691284182, 'right_w1': 1.2954467738891602, 'right_w2': -1.3115535721435547, 'right_e0': 1.3337962935424805, 'right_e1': 1.6463448787170412}

    ATTACK_POS_LEFT_CONST = {'left_w0': 1.0711020839172365, 'left_w1': 1.1731118061950685, 'left_w2': 1.2417574463745118, 'left_e0': -1.2068593834899903, 'left_e1': 1.6494128402893067, 'left_s0': 0.40688840352172856, 'left_s1': -0.4774515196838379}

    DEFENSE_POS_LEFT_CONST = {'left_w0': 0.9802137223388673, 'left_w1': 0.8199127301879884, 'left_w2': 1.5995584647399903, 'left_e0': -0.7113835895690919, 'left_e1': 1.389403097039795, 'left_s0': 0.2695971231628418, 'left_s1': -0.2949078061340332}

    # max and min correspond to goal bounds as percentages of the board width
    MAX_GOAL_WIDTH = 0.73
    MIN_GOAL_WIDTH = 0.27
    
    def __init__(self, game, camera, motion_controller):
        self.camera = camera
        self.motion_controller = motion_controller
        self.game = game

        self.arm = game.get_arm()

        self.tl = tf.TransformListener()
        self.ball_rel_x = 0
        self.ball_rel_y = 0
        self.current_mode = Planner.DEFENSE_MODE
        rospy.Subscriber("/bigredrobot/overhead_camera", OverheadCamera, self.overhead_camera_callback)
    
    def run(self):
        arm = self.motion_controller._arm_obj
        
        window_name = 'Wrist Camera'
        cv.namedWindow(window_name)
        
        stack_pub = rospy.Publisher('/bigredrobot/command', String, queue_size=10)
        
        ## Check phase
        #phase = self.game.get_current_phase() ######### UNCOMMENT FOR COMPETITION
        phase = Game.PHASE_II ###############333######## SET GAME STATE HERE FOR DEBUG ####################
        
        while CALIBRATE:
            raw_input('Press Enter to see joint angles...')
            print self.motion_controller._arm_obj.joint_angles()
            raw_input('Press Enter to see gripper z coordinate...')
            print self.motion_controller.get_gripper_coords()[2]
            rospy.sleep(0.5)
        
        while (phase == Game.PHASE_I or phase == Game.NOT_RUNNING) and not rospy.is_shutdown():
            # nothing happens (wait for phase 2)
            phase = self.game.get_current_phase()
            rospy.sleep(0.5)
            
        while phase == Game.PHASE_II and not rospy.is_shutdown():     
            ## Stack blocks
            rospy.logwarn('PHASE II')
            #phase = self.game.get_current_phase()
    
            stack_pub.publish("scatter")
            rospy.sleep(0.5)
           
        if phase == Game.PHASE_III:  
            rospy.logwarn('PHASE III')   
            # Stop stacking blocks    
            stack_pub.publish("stop")
        
            # Start offense/defenSe
            self.defense_coords = self.enter_defense_mode()
            #self.enter_attack_mode()
            print 'Press <esc> to toggle modes'
            while not rospy.is_shutdown():
                self.raw = cv.blur(camera.get_frame(), (3,3)) 
                
                cv.imshow(window_name, self.raw)
                self.frame = cv.cvtColor(self.raw,cv.COLOR_BGR2HSV)
                cv.waitKey(5)
                
                if self.current_mode == Planner.DEFENSE_MODE:
                    self.defend()
                    self.check_mode()
                else:
                    self.attack(camera)
    
    # Check if we should switch to attack from defense
    def check_mode(self):
        threshold = 0.01
        if self.ball_found:
            if self.arm == 'right':
                if self.ball_rel_y > 0.5 and self.ball_abs_diff < threshold:
                    self.enter_attack_mode()
            else:
                if self.ball_rel_y < 0.5 and self.ball_abs_diff < threshold:
                    self.enter_attack_mode()
            
    ######## ATTACK AND DEFEND FUNCTIONS #############
    def attack(self, camera):
        if self.get_ball(camera):
            self.motion_controller.throw()
            self.defense_coords = self.enter_defense_mode()

    def defend(self):
        pass
        #self.defense_visual_servo()

    def enter_attack_mode(self):
        print 'Entering Attack Mode'
        self.current_mode = Planner.ATTACK_MODE
        self.motion_controller.set_joint_positions(self.ATTACK_POS)

    def enter_defense_mode(self):
        print 'Entering Defense Mode'
        self.current_mode = Planner.DEFENSE_MODE
        #self.motion_controller.set_joint_positions(self.DEFENSE_POS)
        # Return gripper coordinates
        return self.motion_controller.get_gripper_coords()

    ######## VISUAL SERVO FUNCTIONS #############
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
        print 'xi', xi
        vect = Vector3Stamped()
        vect.header.frame_id = '/right_hand_camera'
        vect.header.stamp = rospy.Time(0)
        vect.vector = Vector3(*xi[0:3])
        try:
            trans_vect = self.tl.transformVector3('/base', vect)
            squiggle = np.array([trans_vect.vector.x,trans_vect.vector.y,trans_vect.vector.z,0,0,0])
            print 'squiggle', squiggle
            #motion_controller.command_velocity(squiggle)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "you suck"
        
        return False

    def defense_visual_servo(self):
        current_ball_rel_pos = self.ball_rel_x
        if self.ball_found == False:
            #If we cannot see the ball, stop movement
            motion_controller.command_velocity(np.zeros(6))
            rospy.logwarn('I am doing nothing')
            return False
        # These two lines limit ball_rel_pos to be between the goal posts
        current_ball_rel_pos = min(Planner.MAX_GOAL_WIDTH, current_ball_rel_pos)
        current_ball_rel_pos = max(Planner.MIN_GOAL_WIDTH, current_ball_rel_pos)

        ball_pos = current_ball_rel_pos*(BOARD_MAX_X - BOARD_MIN_X) + BOARD_MIN_X
        hand_pos = np.asarray(self.motion_controller.get_gripper_coords()).squeeze()

        target = np.array([ball_pos, self.defense_coords[1,0], self.defense_coords[2,0]])
        vel = DEFENSE_K0*(target - hand_pos)
        print 'vel: ', vel
        squiggle = np.array([vel[0],vel[1],vel[2],0,0,0])

        try:
            motion_controller.command_velocity(squiggle)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "you suck"
        
        return False

    def overhead_camera_callback(self, data):
        # Get the current position of the ball relative to the width of the board
        # This thing is a percentage of the width
        # Baxter's x and y, not the camera's
        if data.ball_x == -1 or data.ball_y == -1:
            self.ball_found = False
        else:    
            self.ball_found = True
            # (0,0) in rel coords is bottom left in kinect frame
            new_ball_rel_x = ((float(data.center_y)+float(data.height/2)) - data.ball_y)/float(data.height) 
            new_ball_rel_y = 1 - ((float(data.center_x)+float(data.width/2)) - data.ball_x)/float(data.width) 
            
            self.ball_abs_diff = sqrt((new_ball_rel_x - self.ball_rel_x)**2 + (new_ball_rel_y - self.ball_rel_y)**2)
            
            self.ball_rel_x = new_ball_rel_x
            self.ball_rel_y = new_ball_rel_y
            
        print 'ball_rel_pos:', self.ball_rel_x, self.ball_rel_y

    ############## GETTERS
    @property
    def ATTACK_POS(self):
        if self.arm == 'right':
            return Planner.ATTACK_POS_RIGHT_CONST
        else:
            return Planner.ATTACK_POS_LEFT_CONST
                       
    @property
    def DEFENSE_POS(self):
        if self.arm == 'right':
            return Planner.DEFENSE_POS_RIGHT_CONST
        else:
            return Planner.DEFENSE_POS_LEFT_CONST

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
    arm = game.get_arm()

    baxter = Baxter(arm)  
    motion_controller = ml.BaxterMotionController(baxter, arm)
    camera = vl.BaxterCamera(arm)
    planner = Planner(game, camera, motion_controller)

    #debug_with_trackbars()
    planner.run()

