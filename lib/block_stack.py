#!/usr/bin/env python

import rospy
import baxter_interface

from std_msgs.msg import String
from bigredrobot_final.msg import *
from bigredrobot_final.srv import *

class BlockStackController():
    '''
    Create a node to control baxter (via RobotInterface) to stack blocks.
    Start after RobotInterface intialisation.
    
    Example usage:
         try:
            c = BlockStackController()
            c.init_subscribers()
            c.run()
        except rospy.ROSInterruptException:
            pass
    
    '''
    
    def __init__(self):
        rospy.init_node("block_stack_controller", anonymous=True)
        self.command = None        
        self.is_done = False

    def state_update(self, state):
        self.blocks_over = state.blocks_over
        self.gripper_at = state.gripper_at
        self.gripper_closed = state.gripper_closed
        self.state_updated = True


    def command_update(self, command):
        self.command = command.data
        
        if self.command == "stop":
            req = MoveRobotRequest()
            req.action = MoveRobotRequest.DONE_STACKING
            req.target = 0
            self.move_robot(req) # this should stop the robot_interface node    
            # Stop stacking
            self.is_done = True # this will prevent any further service calls from happening
            

    def init_subscribers(self):
        rospy.Subscriber("/bigredrobot/state", State, self.state_update)
        rospy.Subscriber("/bigredrobot/command", String, self.command_update)

    def is_scattered(self):
        if all([x <= 0 for x in self.blocks_over]):
            return True
        else:
            return False    


    def is_stacked_ascending(self):
        if self.blocks_over[0] > 0:
            return False
        for i in range(1,len(self.blocks_over)):
            if self.blocks_over[i] != i:
                return False
        return True
        

    def is_stacked_descending(self):
        if self.blocks_over[-1] > 0:
            return False
        for i in range(len(self.blocks_over)-1):
            if self.blocks_over[i] != i + 2:
                return False
        return True


    def make_available(self, target):
        rospy.loginfo('Make %i available, States: %s' %(target, str(self.blocks_over)))
        #Do the following if the target isn't available yet
        if not self.is_available(target):
            blockOnTop = self.blocks_over.index(target) + 1 #Correct for zero index
            rospy.loginfo('%i is on top of %i' %(blockOnTop,target))
            if not self.is_available(blockOnTop):
                rospy.loginfo('%i is not available' %(blockOnTop))
                self.make_available(blockOnTop)
            rospy.loginfo('Moving %i to %i' %(blockOnTop, -blockOnTop))
            self.move(blockOnTop, -blockOnTop)
        else:
            rospy.loginfo('Block %i is available, States: %s' %(target, str(self.blocks_over)))
        

    def is_available(self, target):
        if target not in self.blocks_over:
            return True
        else:
            return False


    def control_stack_ascending(self):    
        for i in range(1, len(self.blocks_over) + 1):
            self.make_available(i)
            destination = -i if i == 1 else i - 1
            self.move(i, destination)


    def control_stack_descending(self):
        for i in reversed(range(1, len(self.blocks_over) + 1)):
            self.make_available(i)
            destination = -i if i == len(self.blocks_over) else i + 1
            self.move(i, destination)


    def control_scatter(self):
        for i in range(len(self.blocks_over)):
            self.make_available(i+1)


    def control_stack_odd_even(self):
        #For now, stack in any order, but create two separate odd/even stacks
        top_left = -2 # Top block in left stack, initialised to base block
        top_right = -1
        if self.is_stacked_descending():
            for i in range(1, len(self.blocks_over) + 1):
                top_left, top_right = self.split_stack(i, top_left, top_right)

        elif self.is_stacked_ascending():
            for i in reversed(range(1, len(self.blocks_over) + 1)):
                top_left, top_right = self.split_stack(i, top_left, top_right)

        elif self.is_scattered():
            pass

    def split_stack(self, currentblock, top_left, top_right):
        # move a given block on top of block top_left if it is odd, or top right if it is even
        # return the topmost block in the left and right stacks
        if currentblock % 2 == 0:
            self.move(currentblock, top_left)
            top_left = currentblock
        else:
            self.move(currentblock, top_right)
            top_right = currentblock
        return top_left, top_right

    def move(self, blocknum, target):
        # Execute open->move_to->close->move_over sequence
        actions = [MoveRobotRequest.ACTION_OPEN_GRIPPER, MoveRobotRequest.ACTION_MOVE_TO,
                    MoveRobotRequest.ACTION_CLOSE_GRIPPER, MoveRobotRequest.ACTION_MOVE_OVER,
                    MoveRobotRequest.ACTION_OPEN_GRIPPER]
        targets = [lgrip,lblocknum,lgrip,ltarget,lgrip]
        for action, target in zip(actions,targets):
            req = MoveRobotRequest()
            if target is None:
                req.action = MoveRobotRequest.ACTION_IDLE
                req.target = 0
            else:                
                req.action = action
                req.target = target
            rospy.loginfo(req)
            if not self.is_done: # stop sending commands if we are done
                self.move_robot(req)
            else: # throw exception to exit control loop
                raise Exception('stop stacking')
        self.state_updated = False
        while not self.state_updated :
            # Block until state is updated, as this controller requires most recent state from interface
            pass        


    def control(self):
        if self.command == "scatter":
            self.control_scatter()
        elif self.command == "stack_ascending":
            self.control_stack_ascending()
        elif self.command == "stack_descending":
            self.control_stack_descending()
        elif self.command == "stack_odd_even":
            rospy.logwarn('odd_even only available in bimanual mode, try again')
        else:
            rospy.logwarn('You suck at typing. invalid name, try again.')
        self.command = None


    def run(self):
        rospy.wait_for_service('/bigredrobot/move_robot')
        self.move_robot = rospy.ServiceProxy('/bigredrobot/move_robot', MoveRobot)
        try:
            while not self.is_done and not rospy.is_shutdown():                
                if self.command :
                    self.control()
        except Exception as ex:
            print ex # stop node


if __name__ == '__main__':
    try:
        c = BlockStackController()
        c.init_subscribers()
        c.run()
    except rospy.ROSInterruptException:
        pass
