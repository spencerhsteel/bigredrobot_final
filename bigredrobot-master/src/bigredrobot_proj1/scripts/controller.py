#!/usr/bin/env python

## Template for controller node

import rospy
import baxter_interface

from std_msgs.msg import String
from bigredrobot_proj1.msg import *
from bigredrobot_proj1.srv import *

class Controller():
    
    def __init__(self):
        rospy.init_node("controller", anonymous=True)
        self.command = None        

    def state_update(self, state):
        self.blocks_over = state.blocks_over
        self.gripper_at = state.gripper_at
        self.gripper_closed = state.gripper_closed
        self.num_arms = state.num_arms
        self.state_updated = True


    def command_update(self, command):
        self.command = command.data


    def init_subscribers(self):
        rospy.Subscriber("state", State, self.state_update)
        rospy.Subscriber("command", String, self.command_update)


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
        # left arm moves even numbered blocks
        if currentblock % 2 == 0:
            self.move_left(currentblock, top_left)
            top_left = currentblock
        else:
            self.move_right(currentblock, top_right)
            top_right = currentblock
        return top_left, top_right

    def move(self, blocknum, target):
        if self.num_arms == 1 or blocknum % 2 == 1:
            self.move_right(blocknum, target)
        else:
            self.move_left(blocknum, target)

    def move_left(self, blocknum, target):
        self.bimanual_move(blocknum, target, None, None)

    def move_right(self, blocknum, target):
        self.bimanual_move(None, None, blocknum, target)

            
    def bimanual_move(self, lblocknum, ltarget, rblocknum, rtarget):
        # Execute open->move_to->close->move_over sequence in both arms, simultaneously moving blocks lblocknum and rblocknum on top of ltarget and rtarget, respectively. When block and target values are both None the respective arm will be idle.
        actions = [MoveRobotRequest.ACTION_OPEN_GRIPPER, MoveRobotRequest.ACTION_MOVE_TO,
                    MoveRobotRequest.ACTION_CLOSE_GRIPPER, MoveRobotRequest.ACTION_MOVE_OVER,
                    MoveRobotRequest.ACTION_OPEN_GRIPPER]
        lgrip, rgrip = ltarget, rtarget # if target is none ensure gripper does not actuate
        targets = [[lgrip,rgrip], [lblocknum, rblocknum], [lgrip,rgrip], [ltarget, rtarget], [lgrip,rgrip]]
        for action, target in zip(actions,targets):
            req = MoveRobotRequest()
            for arm in [MoveRobotRequest.LEFT_ARM, MoveRobotRequest.RIGHT_ARM]:               
                if target[arm] is None:
                    req.action[arm] = MoveRobotRequest.ACTION_IDLE
                    req.target[arm] = 0
                else:                
                    req.action[arm] = action
                    req.target[arm] = target[arm]
            rospy.loginfo(req)
            self.move_robot(req)
        self.state_updated = False
        while not self.state_updated:
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
            if self.num_arms == 2:
                self.control_stack_odd_even()
            else:
                rospy.logwarn('odd_even only available in bimanual mode, try again')
        else:
            rospy.logwarn('You suck at typing. invalid name, try again.')
        self.command = None


    def run(self):
        self.pub = rospy.Publisher('debug_out', String, queue_size=10)
        rospy.wait_for_service('move_robot')
        self.move_robot = rospy.ServiceProxy('move_robot', MoveRobot)
        
        while not rospy.is_shutdown():                
            if self.command :
                self.control()


if __name__ == '__main__':
    try:
        c = Controller()
        c.init_subscribers()
        c.run()
    except rospy.ROSInterruptException:
        pass
