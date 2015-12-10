#!/usr/bin/env python

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

import baxter_interface

from baxter_interface import CHECK_VERSION

class BaxterMotionController:
    '''
    velocity control for baxter, including line/spline following functionality
    '''

    KP = 0.8                # Line following proportional gain
    MOVE_SPEED = 0.07       # Velocity used when following the line
    K0 = 0                # Gain for the secondary objective function


    def __init__(self, baxter, arm):

        self.arm = arm # arm string
        self._arm_obj = baxter.arm # arm object
        self._joint_names = self._arm_obj.joint_names()
        self._kin = baxter_kinematics(arm)

        self.lower_limits, self.upper_limits = hf.get_limits(self.arm)
        print self.lower_limits
        print self.upper_limits
        print {joint:(self.lower_limits[joint] + self.upper_limits[joint])/2 for joint in self.lower_limits}
        print  self._arm_obj.joint_angles()

         # set joint state publishing to 500Hz
        self.pub_rate = 500.0  # Hz
        self.rate_publisher = rospy.Publisher('robot/joint_state_publish_rate',
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
        '''
        squiggle = np.matrix(squiggle).T
        J = self._kin.jacobian()
        Jinv = hf.rpinv(J)
        q_dot = Jinv*squiggle + (np.identity(7) - (Jinv*J))*self.get_b(self.K0) 
        cmd = {joint: q_dot[i, 0] for i, joint in enumerate(self._joint_names)}
        self._arm_obj.set_joint_velocities(cmd)

    def get_b(self, k):
        '''
        Secondary objective function, designed to avoid joint limits
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
        Follows a straight line in the workspace using proportional control
        '''
        rate = rospy.Rate(self.pub_rate)
        t0 = rospy.Time.now()
        t = 0
        p1 = p1.squeeze()
        p2 = p2.squeeze()
        p12 = p2 - p1 # relative position of p1 wrt p2
        max_dist = np.linalg.norm(p12)
        max_dist_actual = self.dist_from_point(p2)
        v12 = p12/np.linalg.norm(p12)*v0

        rospy.loginfo("Moving. Press Ctrl-C to stop...")
        while (self.dist_from_point(p1) < max_dist_actual) and not rospy.is_shutdown():
            self.rate_publisher.publish(self.pub_rate)
            t = (rospy.Time.now() - t0).to_sec()
            p_estimate = p1 + t*v12
            p_actual = self.get_gripper_coords().squeeze()
            error = p_estimate - p_actual
            v_correct = kp*error
            v_command = v12 + v_correct
            squiggle = np.concatenate((np.matrix(v_command).T, np.matrix([0,0,0]).T))
            self.command_velocity(squiggle)
            rate.sleep()
        rospy.loginfo("Done moving this line")
        self.command_velocity(np.matrix([0,0,0,0,0,0]).T);        
      

