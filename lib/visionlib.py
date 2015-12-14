import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from baxter_interface import camera
import helper_functions as hf


UPPER_PINK1 = [179,220,255]
LOWER_PINK1 = [150,100,5]

class BaxterCamera:
    def __init__(self, arm='right'):
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 400
        
        if arm == 'left':
            cam_name = 'left_hand_camera'
        else:
            cam_name = 'right_hand_camera'

        # arm camera params (right arm values, based on /cameras/.../camera_info_std)
        self.fx = 397.22543258146163
        self.fy = 397.9222130030984
        self.cx = 602.7031818165118
        self.cy = 415.42196609153905

        self.intrinsics = {'fx':self.fx, 'fy':self.fy, 'cx':self.cx, 'cy':self.cy}

        self.cam_ctrl = camera.CameraController(cam_name)
        self.cam_ctrl.resolution = (640,400)
        self.cam_ctrl.exposure = 50 # set exposure 0-100 or cam_ctrl.CONTROL_AUTO
        self.cam_ctrl.gain = 20 # set gain 0-79 or cam_ctrl.CONTROL_AUTO

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/%s/image" %(cam_name), Image, self.capture_callback)
        self.frame = None


    def capture_callback(self,data):
       try:
         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
       except CvBridgeError, e:
         print e
       self.frame = cv_image
       cv.waitKey(3)

    def get_frame(self):
        while self.frame is None and not rospy.is_shutdown():
            pass
        return self.frame
                

    def img_xy_from_uv(self, u, v):
        intr = self.intrinsics
        x = (u - intr['cx']) / intr['fx']
        y = (v - intr['cy']) / intr['fy']
        return x, y

    def interaction_matrix(self, x, y, Z):
        L = np.array([[-1/Z, 0, x/Z, x*y, -(1+x**2), y],
                      [0, -1/Z, y/Z, 1+y**2, -x*y, -x]])
        return L

    def calc_desired_feat_velocities(self, u, v, k0=0.001):
        # velocity point toward image center from current feature point
        #center = np.array([self.FRAME_WIDTH/2, self.FRAME_HEIGHT/2])
        center = np.array([self.FRAME_WIDTH/2 + self.FRAME_WIDTH*0.05, 70])
        current = np.array([u, v])
        velocity = k0*(center-current)
        # print 'center:', center
        # print 'current:', current
    	# This will break interaction matrix code - velocity is a numpy array
        return velocity
        
    def calc_desired_depth_velocity(self, k0, current_pixel_area):
        # Temporarily try and get the ball to be half of the frame width
        # This will need to be replaced with a real number later on.
    	#desired_pixel_area = self.FRAME_WIDTH/3*self.FRAME_HEIGHT/3
        desired_pixel_area = 25000
        velocity = k0*(current_pixel_area - desired_pixel_area)/1000
        # negative so that Baxter moves down towards the ball
        return np.array([velocity])
    
    def calc_desired_depth_velocity_z(self, k0, current_z):
        # Temporarily try and get the ball to be half of the frame width
        # This will need to be replaced with a real number later on.
    	#desired_pixel_area = self.FRAME_WIDTH/3*self.FRAME_HEIGHT/3
        desired_z = -0.07837517
        velocity = k0*(desired_z - current_z)
        # negative so that Baxter moves down towards the ball
        return np.array([velocity])

    # Defecated        
    def calc_end_velocities(self, u, v, Z):
        x, y = self.img_xy_from_uv(u, v)
        L = self.interaction_matrix(x, y, Z)
        s = self.calc_desired_feat_velocities(u, v)
        # print 'Feat vels:', s
        return np.array(hf.rpinv(L)*s).flatten()

class OverheadCamera:
    def __init__(self, USE_DEPTH_REGISTERED):
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 400

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.capture_callback)
        if USE_DEPTH_REGISTERED:
            self.image_sub_d = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback)
        else:
            self.image_sub_d = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        self.frame = None
        self.depthframe = None

    def capture_callback(self,data):
       try:
         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
       except CvBridgeError, e:
         print e
       self.frame = cv_image
       cv.waitKey(3)

    def depth_callback(self,data):
       try:
         depth_image = self.bridge.imgmsg_to_cv2(data, "8UC1")
       except CvBridgeError, e:
         print e
       self.depthframe = depth_image
       cv.waitKey(3)

    def get_frame(self):
        while self.frame is None and not rospy.is_shutdown():
            pass
        return self.frame

    def get_depth_frame(self):
        while self.depthframe is None and not rospy.is_shutdown():
            pass
        return self.depthframe


def track_object(mask):
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(5,5))
    morphed_mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    contours, hierarchy = cv.findContours(morphed_mask.copy(), cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
    max_area = 0
    best_cnt = None
    area = -1
    for cnt in contours:
      area = cv.contourArea(cnt)
      if area > max_area:
        max_area = area
        best_cnt = cnt

    if best_cnt is None or max_area<50:
      x = -1
      y = -1
    else:     
      moment = cv.moments(best_cnt)
      area = moment['m00']
      x = int(moment['m10']/area)
      y = int(moment['m01']/area) 
    return x, y, morphed_mask, area
    
def locate_orange_ball(frame):
    # returns: orng_x, orng_y, mask1, current_area
    #frame = cv.blur(frame, (3,3))
    lower_hsv_orng = np.array([0,0,230])
    upper_hsv_orng = np.array([15,50,255])
    mask_orng = cv.inRange(frame,lower_hsv_orng, upper_hsv_orng)
            
    return track_object(mask_orng)
    
def locate_pink_ball(frame):
    # returns: pink_x, pink_y, mask2, current_area
    frame = cv.blur(frame, (3,3))
    lower_hsv_pink1 = np.array(LOWER_PINK1)
    upper_hsv_pink1 = np.array(UPPER_PINK1)
    mask_pink = cv.inRange(frame,lower_hsv_pink1, upper_hsv_pink1)
    
    #lower_hsv_pink2 = np.array([0, 0, 230])
    #upper_hsv_pink2 = np.array([15, 50, 255])
    #mask_pink2 = cv.inRange(frame, lower_hsv_pink2, upper_hsv_pink2)
    
    #mask_pink = cv.bitwise_or(mask_pink1, mask_pink2)
    
    return track_object(mask_pink)
    

if __name__=="__main__":
    v = BaxterCamera()
    v.run()
