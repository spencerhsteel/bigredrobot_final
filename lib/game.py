import rospy
import cv2 as cv
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from game_server.srv import Init
from game_server.msg import GameState
import rospkg

from bigredrobot_final.srv import Trigger, TriggerResponse


TEST_DEBUG = False
TEST_ARM = 'right'

class Game:
    '''
    Class to communicate with the game server (send logo/teamname, get arm assignment, keep track of phase)
    '''

    # GAME PHASES
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
        
        #logo = np.zeros((300,300,3), np.uint8)
        #logo[:,0:150] = (255,0,0)
        #logo[:,150:300] = (0,255,0) # dummy logo

        rospack = rospkg.RosPack()
        path = rospack.get_path('bigredrobot_final')
        #raise ValueError(path+'/lib/logo.png')
        logo = cv.imread(path + '/lib/logo.png', cv.IMREAD_COLOR)
        if np.random.random(1) < 0.5:
            logo = cv.flip(logo,1)
        if np.random.random(1) < 0.5:
            logo = cv.flip(logo,0)
        rand = np.random.random(1)
        rows, cols, _ = logo.shape
        if rand < 0.25:
            R = cv.getRotationMatrix2D((cols/2,rows/2),0,1)
        elif rand < 0.5:
            R = cv.getRotationMatrix2D((cols/2,rows/2),90,1)
        elif rand < 0.75:
            R = cv.getRotationMatrix2D((cols/2,rows/2),180,1)
        else:
            R = cv.getRotationMatrix2D((cols/2,rows/2),270,1)
        logo = cv.warpAffine(logo, R, (cols,rows))
       
        
        # Convert to imgmsg
        logo_msg = bridge.cv2_to_imgmsg(logo, encoding="bgr8")

        # Call init service of game_server and get arm
        if TEST_DEBUG:
            self.arm = TEST_ARM
        else:
            init = rospy.ServiceProxy('/game_server/init', Init)
            response = init("bigredrobot", logo_msg)
            self.arm = response.arm
            
            # Subscribe to game_state updates
            rospy.Subscriber('/game_server/game_state', GameState, self.game_state_callback)


        # Initialise a service for use by other nodes to get arm (returns True=right, False=left)
        rospy.Service('/bigredrobot/arm', Trigger, self.arm_callback) 


    def get_arm(self):
        return self.arm

    def arm_callback(self, req):
        return TriggerResponse(success=True if self.arm=='right' else False, message="booooo")

    def get_current_phase(self):
        return self.current_phase # 0, 1, 2, or 3 (0 indicates game not running)
        
    def get_time_remaining(self):
        return self.time_remaining # return duration type
        
    def get_update_time(self):
        return self.update_time # returns time that the state was last updated

    def game_state_callback(self, msg):
        if self.arm is None:
            return
        self.current_phase = msg.current_phase
        self.time_remaining  = msg.time_remaining
        self.update_time = rospy.Time.now()
        if self.arm == 'left':	
            self.score = msg.score[msg.LEFT]
            self.opponent_score = msg.score[msg.RIGHT]
        else:
            self.score = msg.score[msg.RIGHT]
            self.opponent_score = msg.score[msg.LEFT]
        print 'phase:', self.current_phase
        print 'time remaining:', self.get_time_remaining()


if __name__ == '__main__':
    rospy.init_node('game_server_test')
    game = Game()
    print game.get_arm()
    while not rospy.is_shutdown():
            pass
    print game.current_phase
