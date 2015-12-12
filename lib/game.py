import rospy
import cv2 as cv
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from game_server.srv import Init
from game_server.msg import GameState


class Game:


    NOT_RUNNING = 0
    PHASE_I = 1
    PHASE_II = 2
    PHASE_III = 3


    def __init__(self):
        bridge = CvBridge()
        
        self.current_phase = Game.PHASE_I
        self.time_remaining = None
        self.score = None
        self.opponent_score = None
        self.update_time = rospy.Time.now()

        # load 300x300 logo
        logo = np.zeros((300,300,3), np.uint8)
        logo[:,0:150] = (255,0,0)
        logo[:,150:300] = (0,255,0)
        # Convert to imgmsg
        logo_msg = bridge.cv2_to_imgmsg(logo, encoding="rgb8")

        # Call init service of game_server and get arm
        init = rospy.ServiceProxy('/game_server/init', Init)
        response = init("bigredrobot", logo_msg)
        self.arm = response.arm

        # Subscribe to game_state updates
        rospy.Subscriber('/game_server/game_state', GameState, self.game_state_callback)

    def get_arm(self):
        return self.arm

    def get_current_phase(self):
        return self.current_phase # 0, 1, 2, or 3 (0 indicates game not running)
        
    def get_time_remaining(self):
        return self.time_remaining.data # return duration type
        
    def get_update_time(self):
        return self.update_time # returns time that the state was last updated

    def game_state_callback(self, msg):
        self.current_phase = msg.current_phase
        self.time_remaining  = msg.time_remaining
        self.update_time = rospy.Time.now()
        if arm == 'left':	
            self.score = msg.score[msg.LEFT]
            self.opponent_score = msg.score[msg.RIGHT]
        else:
            self.score = msg.score[msg.RIGHT]
            self.opponent_score = msg.score[msg.LEFT]


if __name__ == '__main__':
    game = Game()
    print game.get_arm()
    while game.current_phase is None:
        pass
    print game.current_phase
