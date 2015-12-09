#!/usr/bin/env python

## Template for robot_interface node

import rospy
from bigredrobot_proj1.srv import *
from bigredrobot_proj1.msg import *

#Should probably go to robot_interface
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
    
    def __init__(self):
        rospy.init_node('robot_interface', anonymous=True)
        self.is_real_robot = not rospy.get_param('symbolic_only')
        self.num_arms = rospy.get_param('num_arms')
        if self.is_real_robot:
            # initialise baxter
            rospy.logwarn('baxter is online')
            baxter_interface.RobotEnable().enable()
            self.right_limb = baxter_interface.Limb('right') 
            self.right_gripper = baxter_interface.Gripper('right')
            #self.right_gripper.calibrate()
            self.right_gripper.open()
            self.right_gripper.set_holding_force(100)

            if self.num_arms == 2:
                self.left_limb = baxter_interface.Limb('left') 
                self.left_gripper = baxter_interface.Gripper('left')
                #self.left_gripper.calibrate()
                self.left_gripper.open()
                self.left_gripper.set_holding_force(100)
            
            self.TABLE_Z = -0.08
            self.BLOCK_HEIGHT = 0.045

            self.ik_solve_right = rospy.ServiceProxy("ExternalTools/right/PositionKinematicsNode/IKService", SolvePositionIK)
            self.ik_solve_left = rospy.ServiceProxy("ExternalTools/left/PositionKinematicsNode/IKService", SolvePositionIK)


    def init_state(self):
        self.num_blocks = rospy.get_param('num_blocks')
        num_blocks = self.num_blocks # We suck
        configuration = rospy.get_param('configuration')
        if self.num_arms == 1:
            self.arms = [State.RIGHT_ARM] # If we are only using one arm. We are using right        
        elif self.num_arms == 2:
            self.arms = [State.LEFT_ARM, State.RIGHT_ARM]
        else:
            raise ValueError('Wrong number of arms')
     
        self.gripper_at = [None]*2
        self.gripper_closed = [None]*2
        
        # convention: every block has a corresponding 'virtual' block reffered to by -blocknum
        if configuration=='scattered':
            # Symbolic only
            self.blocks_over = list(range(-num_blocks+1,1)) # e.g [-2, -1, 0]
            self.gripper_at[State.LEFT_ARM] = -(num_blocks - (num_blocks%2)) 
            self.gripper_at[State.RIGHT_ARM] = num_blocks
        elif configuration=='stacked_ascending':
            self.blocks_over = list(range(num_blocks)) # e.g [0, 1, 2]
            self.blocks_over[0] = 0
            self.gripper_at[State.LEFT_ARM] = -(num_blocks - (num_blocks%2)) 
            self.gripper_at[State.RIGHT_ARM] = num_blocks
        elif configuration=='stacked_descending':
            self.blocks_over = list(range(2,num_blocks+1)) # e.g [2, 3, 0]
            self.blocks_over.append(0)
            self.gripper_at[State.LEFT_ARM] = -(num_blocks - (num_blocks%2)) 
            self.gripper_at[State.RIGHT_ARM] = 1
            self.gripper_closed[State.LEFT_ARM] = True   
            self.gripper_closed[State.RIGHT_ARM] = True  
           
        if self.is_real_robot:
            # augment world state with coordinates
            pose = self.right_limb.endpoint_pose()
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
            nextblock = self.gripper_at[State.RIGHT_ARM]
            nextz = pos.z
            while nextblock > 0:
                # TODO: make sure we always start on block 0
                self.block_coords[nextblock] = [self.base_x, self.base_y[0], nextz]
                nextz = nextz - self.BLOCK_HEIGHT
                nextblock = self.blocks_over[nextblock-1]
            self.ORIENT = pose.popitem()[1]
            

    def init_publisher(self):
        self.pub = rospy.Publisher('state', State, queue_size=10)

    # Callback function for move_robot server
    def handle_move_robot(self, req):    
        for arm in self.arms:
            if req.action[arm]==req.ACTION_OPEN_GRIPPER:
                if self.gripper_closed[arm]:
                    self.robot_open_gripper(arm)
                    self.gripper_closed[arm] = False
                else:
                    rospy.logwarn('Invalid action OPEN_GRIPPER (arm = %i)' %(arm))
                    return False
            elif req.action[arm]==req.ACTION_CLOSE_GRIPPER:
                if not self.gripper_closed[arm]:
                    self.robot_close_gripper(arm)
                    self.gripper_closed[arm] = True
                else:
                    rospy.logwarn('Invalid action CLOSE_GRIPPER(arm = %i)' %(arm))
                    return False
            elif req.action[arm]==req.ACTION_MOVE_TO:
                if not self.gripper_closed[arm] and self.is_topmost(req.target[arm]):
                    self.robot_move_to(req.target[arm], arm)
                    self.gripper_at[arm] = req.target[arm]
                else:
                    rospy.logwarn('Invalid action MOVE_TO (arm = %i, target = %i)' %(arm, req.target[arm]))
                    return False
            elif req.action[arm]==req.ACTION_MOVE_OVER:
                if self.gripper_closed[arm] and self.is_topmost(req.target[arm]):
                    self.robot_move_over(req.target[arm], arm)
                    self.blocks_over[self.gripper_at[arm]-1] = req.target[arm]
                else:
                    rospy.logwarn('Invalid action MOVE_OVER (arm = %i, target = %i, current block = %i)' %(arm, req.target[arm], self.gripper_at[arm]))
                    return False
            elif req.action[arm]==req.ACTION_IDLE:
                pass # nothing to see here
        return False


# Real robot move functions.
# Send commands to baxter 
# Must be called before symbolic state update        
    def robot_open_gripper(self, arm):
        if self.is_real_robot:
            if arm == State.RIGHT_ARM:
                self.right_gripper.open(block=True)
            else:
                self.left_gripper.open(block=True)

    def robot_close_gripper(self, arm):
        if self.is_real_robot:
            if arm == State.RIGHT_ARM:
                self.right_gripper.close(block=True)
            else:
                self.left_gripper.close(block=True)

    def get_gripper_coords(self, arm):
        if arm == State.LEFT_ARM:
            pose = self.left_limb.endpoint_pose()
        else:
            pose = self.right_limb.endpoint_pose()
        pos = pose.popitem()[1]
        return pos.x, pos.y, pos.z

    def avoid_conflict(self, target, arm):
            other_arm = (arm+1) % 2
            if True: # Move non-commanded arm to safe position at every action
                rospy.logwarn('avoid conflict, arm = %i, other_arm=%i, target=%i' %(arm, other_arm, target))
                ycoords = [coord[1] for coord in self.block_coords.values()]
                if other_arm == State.LEFT_ARM:
                    pose = self.left_limb.endpoint_pose()
                    pos = pose.popitem()[1]
		    # Move to some point in the workspace very far from the center
                    x1, y1, z1 = pos.x, max(ycoords) + 3*self.BLOCK_HEIGHT, self.base_z + (self.num_blocks+1.5)*self.BLOCK_HEIGHT
                else:
                    pose = self.right_limb.endpoint_pose()
                    pos = pose.popitem()[1]
                    x1, y1, z1 = pos.x, min(ycoords) - 3*self.BLOCK_HEIGHT, self.base_z + (self.num_blocks+1.5)*self.BLOCK_HEIGHT          
                self.move_safely(pos.x,pos.y,pos.z,x1,y1,z1,other_arm)

    def robot_move_to(self, target, arm):
        if self.is_real_robot: 
            rospy.logwarn('move to, arm = %i, target = %i' %(arm, target))
            if self.num_arms==2:            
                self.avoid_conflict(target,arm)
            x0, y0, z0 = self.get_gripper_coords(arm) 
            x1, y1, z1 = self.block_coords[target]  
            self.move_safely(x0,y0,z0,x1,y1,z1,arm)


    def robot_move_over(self, target, arm):
        if self.is_real_robot: 
            rospy.logwarn('move over, arm = %i, target = %i' %(arm, target))
            if self.num_arms==2:                        
                self.avoid_conflict(target,arm)
            x0, y0, z0 = self.get_gripper_coords(arm)      
            x1, y1, z1_ = self.block_coords[target]  
            z1 = z1_ + self.BLOCK_HEIGHT            
            self.move_safely(x0,y0,z0,x1,y1,z1,arm)
            self.block_coords[self.gripper_at[arm]] = [x1, y1, z1]
    
    def move_safely(self,x0,y0,z0,x1,y1,z1,arm):
	    # Move safely between source and destination coords by moving vertically above maximum stack height first then moving horizontally, followed by a final vertical move to the destination z coord
            self.move_robot(x0, y0, self.base_z + (self.num_blocks+1.5)*self.BLOCK_HEIGHT, arm)
            self.move_robot(x1, y1, self.base_z + (self.num_blocks+1.5)*self.BLOCK_HEIGHT, arm)
            self.move_robot(x1, y1, z1, arm)


    def move_robot(self, x, y, z, arm):
	    # Command arm to move to coordinates x,y,z
            loc = Point(float(x),float(y),float(z))
            hdr = Header(stamp=rospy.Time.now(), frame_id='base')
            ikreq = SolvePositionIKRequest()
            poses = {'right': PoseStamped(header=hdr, pose=Pose(position=loc, orientation=self.ORIENT))}

            ikreq.pose_stamp.append(poses['right'])
            
            if arm == State.RIGHT_ARM:
                resp = self.ik_solve_right(ikreq)
            else:
                resp = self.ik_solve_left(ikreq)

            if not resp.isValid[0]:
                rospy.logwarn("No IK solution")
            else:
                rospy.loginfo("VALID IK")
            rospy.loginfo(resp)
            
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            rospy.loginfo(limb_joints)
            if arm == State.RIGHT_ARM:           
                self.right_limb.move_to_joint_positions(limb_joints)
            else:
                self.left_limb.move_to_joint_positions(limb_joints)

    def init_service(self):
        self.srv = rospy.Service('move_robot', MoveRobot, self.handle_move_robot) 
    
    def run(self):
	rospy.logwarn("Ready to receive commands")
        rate = rospy.Rate(1) # Update state 1hz
        while not rospy.is_shutdown():
            state = State()
            state.blocks_over = self.blocks_over
            state.gripper_at = self.gripper_at
            state.gripper_closed = self.gripper_closed
            state.num_arms = self.num_arms
            self.pub.publish(state)
            rate.sleep()
    
    def is_topmost(self, target):
        if target not in self.blocks_over:
            return True
        else:
            return False

    def baxter_move(source, dest):
        pass
        

if __name__ == '__main__':
    try:
        robint = RobotInterface()
        robint.init_state()
        robint.init_publisher()
        robint.init_service()
        robint.run()

    except rospy.ROSInterruptException:
        pass
