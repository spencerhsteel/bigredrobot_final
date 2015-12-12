#!/usr/bin/env python


import rospy
from bigredrobot_final.srv import *
from bigredrobot_final.msg import *

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface 

class RobotInterface:
    '''
	Create a node to handle MoveRobot service calls RobotInterface and move baxter accordingly.
	Start before BlockStackController intialisation.
	
	Stop node / finish stacking by sending a DONE action via MoveRobot srv call
	
	Example usage:
		try:
			robint = RobotInterface()
			robint.init_state()
			robint.init_publisher()
			robint.init_service()
			robint.run()

		except rospy.ROSInterruptException:
			pass
	
	'''
    def __init__(self, arm):        
        rospy.init_node("robot_interface", anonymous=True)

        self.is_done = False

        # initialise limb and gripper
        self.limb = baxter_interface.Limb(arm) 
        self.gripper = baxter_interface.Gripper(arm)
        self.gripper.open()
        
        self.TABLE_Z = -0.08
        self.BLOCK_HEIGHT = 0.045

        self.ik_solve = rospy.ServiceProxy("ExternalTools/%s/PositionKinematicsNode/IKService"%(arm), SolvePositionIK)


    def init_state(self):
        self.num_blocks = rospy.get_param('/num_blocks')
        num_blocks = self.num_blocks # We suck
        configuration = rospy.get_param('/configuration')
     
        self.gripper_at = 0
        self.gripper_closed = 0
        
        # convention: every block has a corresponding 'virtual' block referred to by -blocknum
        if configuration=='scattered':
            # Symbolic only
            self.blocks_over = list(range(-num_blocks+1,1)) # e.g [-2, -1, 0]
            self.gripper_at = num_blocks
        elif configuration=='stacked_ascending':
            self.blocks_over = list(range(num_blocks)) # e.g [0, 1, 2]
            self.blocks_over[0] = 0
            self.gripper_at = num_blocks
        elif configuration=='stacked_descending':
            self.blocks_over = list(range(2,num_blocks+1)) # e.g [2, 3, 0]
            self.blocks_over.append(0)
            self.gripper_at = 1
            self.gripper_closed = True  
           

        # augment world state with coordinates
        pose = self.limb.endpoint_pose()
        pos = pose.popitem()[1]
        rospy.logwarn(pos)
        self.base_x = pos.x # All block stacks lie on the same x position
        self.base_y = {i:0 for i in range(-self.num_blocks,1)} 
        self.base_z = pos.z - self.num_blocks*self.BLOCK_HEIGHT # Find z corresponding to base block
        self.block_coords = {i:[0, 0, 0] for i in range(-self.num_blocks,self.num_blocks+1)} # [x,y,z]
        for i in self.base_y:
            if i % 2 == 1:
                self.base_y[i] = pos.y + 1.5*(2*i*self.BLOCK_HEIGHT - (i+1)*self.BLOCK_HEIGHT)
            else:
                self.base_y[i] = pos.y - 1.5*i*self.BLOCK_HEIGHT
            self.block_coords[i] = [self.base_x, self.base_y[i], self.base_z]
        rospy.logwarn(self.base_y)
        rospy.logwarn('base x = %f, base y[0] = %f' %(self.base_x, self.base_y[0]))
        nextblock = self.gripper_at
        nextz = pos.z
        while nextblock > 0:
            self.block_coords[nextblock] = [self.base_x, self.base_y[0], nextz]
            nextz = nextz - self.BLOCK_HEIGHT
            nextblock = self.blocks_over[nextblock-1]
        self.ORIENT = pose.popitem()[1]
            

    def init_publisher(self):
        self.pub = rospy.Publisher('state', State, queue_size=10)

    # Callback function for move_robot server
    def handle_move_robot(self, req):    
        if req.action==req.ACTION_OPEN_GRIPPER:
            if self.gripper_closed:
                self.robot_open_gripper()
                self.gripper_closed = False
            else:
                rospy.logwarn('Invalid action OPEN_GRIPPER')
                return False
        elif req.action==req.ACTION_CLOSE_GRIPPER:
            if not self.gripper_closed:
                self.robot_close_gripper()
                self.gripper_closed = True
            else:
                rospy.logwarn('Invalid action CLOSE_GRIPPER')
                return False
        elif req.action==req.ACTION_MOVE_TO:
            if not self.gripper_closed and self.is_topmost(req.target):
                self.robot_move_to(req.target,)
                self.gripper_at = req.target
            else:
                rospy.logwarn('Invalid action MOVE_TO (target = %i)' %( req.target))
                return False
        elif req.action==req.ACTION_MOVE_OVER:
            if self.gripper_closed and self.is_topmost(req.target):
                self.robot_move_over(req.target)
                self.blocks_over[self.gripper_at-1] = req.target
            else:
                rospy.logwarn('Invalid action MOVE_OVER (target = %i, current block = %i)' %(req.target, self.gripper_at))
                return False
        elif req.action==req.ACTION_IDLE:
            pass # nothing to see here
        elif req.action==req.DONE_STACKING:
        	self.is_done = True
        return False


# Real robot move functions.
# Send commands to baxter 
# Must be called before symbolic state update        
    def robot_open_gripper(self):
        self.gripper.open(block=True)


    def robot_close_gripper(self):
        self.gripper.close(block=True)
        
    def get_gripper_coords(self):
        self.limb.endpoint_pose()
        pos = pose.popitem()[1]
        return pos.x, pos.y, pos.z

    def robot_move_to(self, target):
        rospy.logwarn('move to target = %i' %(target))
        x0, y0, z0 = self.get_gripper_coords() 
        x1, y1, z1 = self.block_coords[target]  
        self.move_safely(x0,y0,z0,x1,y1,z1)


    def robot_move_over(self, target):
            rospy.logwarn('move over target = %i' %(target)  )    
            x0, y0, z0 = self.get_gripper_coords()      
            x1, y1, z1_ = self.block_coords[target]  
            z1 = z1_ + self.BLOCK_HEIGHT            
            self.move_safely(x0,y0,z0,x1,y1,z1)
            self.block_coords[self.gripper_at] = [x1, y1, z1]
    
    def move_safely(self,x0,y0,z0,x1,y1,z1):
	    # Move safely between source and destination coords by moving vertically above maximum stack height first then moving horizontally, followed by a final vertical move to the destination z coord
            self.move_robot(x0, y0, self.base_z + (self.num_blocks+1.5)*self.BLOCK_HEIGHT)
            self.move_robot(x1, y1, self.base_z + (self.num_blocks+1.5)*self.BLOCK_HEIGHT)
            self.move_robot(x1, y1, z1)


    def move_robot(self, x, y, z):
	    # Command arm to move to coordinates x,y,z
        loc = Point(float(x),float(y),float(z))
        hdr = Header(stamp=rospy.Time.now(), frame_id='/base')
        ikreq = SolvePositionIKRequest()
        pose = PoseStamped(header=hdr, pose=Pose(position=loc, orientation=self.ORIENT))

        ikreq.pose_stamp.append(pose)
        
        resp = self.ik_solve(ikreq)
       
        if not resp.isValid[0]:
            rospy.logwarn("No IK solution")
        else:
            rospy.loginfo("VALID IK")
        rospy.loginfo(resp)
        
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        self.limb.move_to_joint_positions(limb_joints)
            

    def init_service(self):
        self.srv = rospy.Service('move_robot', MoveRobot, self.handle_move_robot) 
    
    def run(self):
   		self.done = False
		rospy.logwarn("Ready to receive commands")
        rate = rospy.Rate(1) # Update state 1hz
        while not self.is_done and not rospy.is_shutdown():
            state = State()
            state.blocks_over = self.blocks_over
            state.gripper_at = self.gripper_at
            state.gripper_closed = self.gripper_closed
            self.pub.publish(state)
            rate.sleep()
    

if __name__ == '__main__':
    try:
        robint = RobotInterface()
        robint.init_state()
        robint.init_publisher()
        robint.init_service()
        robint.run()

    except rospy.ROSInterruptException:
        pass
