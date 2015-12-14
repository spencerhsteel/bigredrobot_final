#!/usr/bin/env python
import rospy

import numpy as np
import helper_functions as hf
from baxter_pykdl import baxter_kinematics
from scipy import interpolate

from std_msgs.msg import (
    UInt16,
    String
)


class BaxterMotionController:
    '''
    velocity control for baxter, including line/spline following functionality
    '''

    KP = 0.8                # Line following proportional gain
    MOVE_SPEED = 0.07       # Velocity used when following the line
    K0 = 0                # Gain for the secondary objective function

    RIGHT_THROW_START_ANGLES = {'right_s0': -0.6316165886901856, 'right_s1': 0.1967330358215332, 'right_w0': -1.8871798621398927, 'right_w1': 1.553539041156006, 'right_w2': -1.5681118586242677, 'right_e0': 1.7264953747924805, 'right_e1': 1.3625584332824707}
    LEFT_THROW_START_ANGLES = {'left_w0': 1.251728321484375, 'left_w1': 1.392854553808594, 'left_w2': 1.920160449041748, 'left_e0': -1.3445341590454103, 'left_e1': 1.062665189593506, 'left_s0': 0.5330583231811524, 'left_s1': -0.24697090656738283}

    # START ATTACK THROW
    RIGHT_THROW_START_ANGLES = {'right_s0': -0.2791845030761719, 'right_s1': 0.15454856420288088, 'right_w0': -1.7878546062377931, 'right_w1': 1.4641846603637696, 'right_w2': 1.7491215913879397, 'right_e0': 1.6260196333007813, 'right_e1': 1.1217234498596191}


    def __init__(self, baxter, arm):

        self.arm = arm # arm string
        self._arm_obj = baxter.arm_obj # arm object
        self._gripper_obj = baxter.gripper

        self._joint_names = self._arm_obj.joint_names()
        self._kin = baxter_kinematics(arm)

        self.lower_limits, self.upper_limits = hf.get_limits(self.arm)
        print self.lower_limits
        print self.upper_limits
        print {joint:(self.lower_limits[joint] + self.upper_limits[joint])/2 for joint in self.lower_limits}
        print  self._arm_obj.joint_angles()

         # set joint state publishing to 500Hz
        self.pub_rate = 500.0  # Hz
        self.rate_publisher = rospy.Publisher('/robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self.rate_publisher.publish(self.pub_rate)

    ########################################
    # Housekeeping functions
    ########################################

    def _reset_control_modes(self):
        '''
        Resets the joint publish rate and stops writing commands to Baxter
        '''
        rate = rospy.Rate(self.pub_rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._arm_obj.exit_control_mode()
            self.rate_publisher.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True
    
    def set_neutral(self):
        '''
        Sets both arms back into a neutral pose.
        '''
        rospy.loginfo("Moving to neutral pose...")
        self._arm_obj.move_to_neutral()

    def clean_shutdown(self):
        rospy.loginfo("\nCrashing stuff...")
        self._reset_control_modes()
        return True

    ########################################
    # Helper functions
    ########################################

    def set_joint_positions(self,angles):
        self._arm_obj.move_to_joint_positions(angles)

    def get_gripper_coords(self):
        pos = self._arm_obj.endpoint_pose().popitem()[1]
        return np.matrix([pos.x, pos.y, pos.z]).T

    def dist_from_point(self, p):
        '''
        Gets the distance of the gripper from some point p.
        'p' is a numpy column vector
        '''
        gripper = self.get_gripper_coords()
        r = gripper.squeeze() - p.squeeze()
        return np.linalg.norm(r)

    ########################################
    # Velocity control and secondary objective functions
    ########################################

    def command_velocity(self, squiggle):
        '''
        Commands joint velocities using jacobian
        Squiggle is an np.array (1D)!!!!!!
        '''
        squiggle = np.matrix(squiggle).T
        J = self._kin.jacobian()
        Jinv = hf.rpinv(J)
        q_dot = Jinv*squiggle + (np.identity(7) - (Jinv*J))*self.get_b(self.K0) 
        cmd = {joint: q_dot[i, 0] for i, joint in enumerate(self._joint_names)}
        self._arm_obj.set_joint_velocities(cmd)

    def get_b(self, k):
        '''
        Secondary objective function, designed to avoid joint{'left_w0': 1.7391507162780764, 'left_w1': 1.2371555040161133, 'left_w2': 1.499082723248291, 'left_e0': -1.3330293031494143, 'left_e1': 1.3372477503112794, 'left_s0': 0.5326748279846192, 'left_s1': 0.20248546376953128}
 limits
        https://books.google.com/books?id=nyrY0Pu5kl0C&pg=PA128&dq=jacobian+secondary+objective+function&hl=en&sa=X&ved=0ahUKEwiqscikhM_JAhVljIMKHZZ7CYQQ6AEIHDAA#v=onepage&q=jacobian%20secondary%20objective%20function&f=false
        '''
        joint_angles = self._arm_obj.joint_angles()

        q = []
        q_bar = []
        delta_q = []


        delta_q = []
        for joint in hf.get_frame_dict(self.arm):
            q.append(joint_angles[joint])
            q_bar.append((self.lower_limits[joint] + self.upper_limits[joint])/2)
            delta_q.append(self.upper_limits[joint] - self.lower_limits[joint])
            
            
        q = np.array(q)
        q_bar = np.array(q_bar)
        delta_q = np.array(delta_q) 
        
        z = (q_bar - q)/np.square(delta_q)
        
        #print 'qb:', q_bar    
        #print 'z:', z
        #print 'q', q
        #print '\n'
        
        return -k * np.matrix(z).T

    ########################################
    # Line and spline following functions
    ########################################
    def follow_spline(self, tck, u, u_step=0.1):
        '''
        Follows a spline in the workspace
        '''
        u_min = min(u)
        u_max = max(u)
        new_u = np.arange(u_min, u_max+u_step, u_step)
        out = interpolate.splev(new_u, tck)
        x = out[0]
        y = out[1]
        z = out[2]
        for i in range(len(new_u)-1):
            p1 = np.array([x[i], y[i], z[i]])
            p2 = np.array([x[i+1], y[i+1], z[i+1]])
            self.follow_line_p_control(p1, p2, self.MOVE_SPEED, self.KP)

    def follow_line_p_control(self, p1, p2, v0, kp):
        '''
        p1, p2 are 1D np arrays!!!!!!!!!!!!!!!!1
        Follows a straight line in the workspace using proportional control
        '''
        rate = rospy.Rate(self.pub_rate)
        t0 = rospy.Time.now()
        t = 0
        p12 = p2 - p1 # relative position of p1 wrt p2
        max_dist = np.linalg.norm(p12)
        max_dist_actual = self.dist_from_point(p2)
        v12 = p12/np.linalg.norm(p12)*v0

        rospy.loginfo("Moving. Press Ctrl-C to stop...")
        while (self.dist_from_point(p1) < max_dist_actual) and not rospy.is_shutdown():
            self.rate_publisher.publish(self.pub_rate)
            t = (rospy.Time.now() - t0).to_sec()
            p_estimate = p1 + t*v12
            p_actual = np.asarray(self.get_gripper_coords()).squeeze()
            error = p_estimate - p_actual
            v_correct = kp*error
            v_command = v12 + v_correct
            squiggle = np.concatenate((np.matrix(v_command), np.matrix([0,0,0])),axis=1)
            self.command_velocity(squiggle)
            rate.sleep()
        rospy.loginfo("Done moving this line")
        
        # Commented this out because its no good for throwing
        #self.command_velocity(np.matrix([0,0,0,0,0,0]).T);   

    ########################################
    # Throwing functions
    ########################################
    def throw(self, throw_vel=2, throw_dist=0.2):
        if self.arm == 'right':
            start_angles = self.RIGHT_THROW_START_ANGLES
        else:
            start_angles = self.LEFT_THROW_START_ANGLES
            throw_dist = -throw_dist # throw in the opposite direction

        # move to throw start position
        self._arm_obj.move_to_joint_positions(start_angles)

        p_0 = np.asarray(self.get_gripper_coords().T).squeeze() # as np.array([x, y, z])
        delta_p_throw = np.array([0.05, throw_dist, 0]) # throw along the negative y for the right arm
        p_1 = p_0 + delta_p_throw
        p_2 = p_1 + delta_p_throw
        
        kp = self.KP*2
        self.follow_line_p_control(p_0, p_1, throw_vel, kp) ####INCREASED GAIN BY 1.5x
        self._gripper_obj.open() # bring the pain
        self.follow_line_p_control(p_1, p_2, throw_vel, kp)
        

    def move_up(self, vel, dist):

        p_0 = np.asarray(self.get_gripper_coords().T).squeeze() 
        delta_p_throw = np.array([0, 0, dist]) 
        p_1 = p_0 + delta_p_throw
        
        self.follow_line_p_control(p_0, p_1, vel, self.KP) 
        
        
        
               
      

