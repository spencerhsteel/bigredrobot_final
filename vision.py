import cv2 as cv
import numpy as np
import rospy
import std_srvs.srv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from baxter_interface import camera
import helper_functions as hf
import baxter_interface
from baxter_interface import CHECK_VERSION

# http://www.pages.drexel.edu/~nk752/Visual_Servoing_Article_Part_1.pdf
class Vision:
    def __init__(self):
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 400
        self.capture = cv.VideoCapture(0)
        self.capture.set(4, self.FRAME_WIDTH)
        self.capture.set(5, self.FRAME_HEIGHT)  
        self.MAX_NUM_OBJECTS = 50
        self.MIN_OBJECT_AREA = 15*15
        self.MAX_OBJECT_AREA = self.FRAME_HEIGHT*self.FRAME_WIDTH/1.5
        
        self.cam_ctrl = camera.CameraController('right_hand_camera')
        #self.cam_ctrl.exposure(50) # set exposure 0-100 or cam_ctrl.CONTROL_AUTO

        # arm camera params (right arm values, based on /cameras/.../camera_info_std)
        self.fx = 397.22543258146163
        self.fy = 397.9222130030984
        self.cx = 602.7031818165118
        self.cy = 415.42196609153905

        self.intrinsics = {'fx':self.fx, 'fy':self.fy, 'cx':self.cx, 'cy':self.cy}

        rospy.ServiceProxy('/cameras/close \'left_hand_camera\'', std_srvs.srv.Empty)
        rospy.ServiceProxy('/cameras/open \'{name: right_hand_camera, settings: {width: 640, height: 400}}\'', std_srvs.srv.Empty)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,self.callback)

    def img_xy_from_uv(self, u, v):
        intr = self.intrinsics
        x = (u - intr['cx']) / intr['fx']
        y = (v - intr['cy']) / intr['fy']
        return x, y

    def interaction_matrix(self, x, y, Z):
        L = np.array([[-1/Z, 0, x/Z, x*y, -(1+x**2), y],
                         [0, -1/Z, y/Z, 1+y**2, -x*y, -x]])
        return L

    def calc_desired_feat_velocities(self, u, v, k0=1):
        # velocity point toward image center from current feature point
        center = np.array([self.FRAME_WIDTH/2, self.FRAME_HEIGHT/2])
        current = np.array([u, v])
        velocity = k0*(center-current)
        print 'center:', center
        print 'current:', current
        return np.matrix(velocity).T
        
    def calc_end_velocities(self, u, v, Z):
        x, y = self.img_xy_from_uv(u, v)
        L = self.interaction_matrix(x, y, Z)
        s = self.calc_desired_feat_velocities(u, v)
        print 'Feat vels:', s
        return hf.rpinv(L)*s

    def callback(self,data):
       try:
         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
       except CvBridgeError, e:
         print e
       self.frame = cv_image
       cv.waitKey(3)

    def on_trackbar(self,value):
        return 0

    def trackFilteredObject(self,oldtemp):
        # RENAME EVERYTHING
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(5,5))
        temp = cv.morphologyEx(oldtemp, cv.MORPH_OPEN, kernel)
        contours, hierarchy = cv.findContours(temp.copy(), cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
        max_area = 0
        best_cnt = None
        for cnt in contours:
          area = cv.contourArea(cnt)
          if area > max_area:
            max_area = area
            best_cnt = cnt

        if best_cnt is None or max_area<100:
          x = -1
          y = -1
        else:     
          moment = cv.moments(best_cnt)
          area = moment['m00']
          x = int(moment['m10']/area)
          y = int(moment['m01']/area) 
        return x, y, temp  # temp is the morph-opened image   

    def run(self):
        rospy.init_node('VisionTest', anonymous=True)
        
        # Creating a window for later use
        cv.namedWindow('result')

        # Creating track bar
        cv.createTrackbar('hmin', 'result',0,179,self.on_trackbar)
        cv.createTrackbar('hmax', 'result',179,179,self.on_trackbar)
        cv.createTrackbar('smin', 'result',0,255,self.on_trackbar)
        cv.createTrackbar('smax', 'result',255,255,self.on_trackbar)
        cv.createTrackbar('vmin', 'result',0,255,self.on_trackbar)
        cv.createTrackbar('vmax', 'result',255,255,self.on_trackbar)

        while not rospy.is_shutdown():

            #_, self.frame = self.capture.read()
            self.frame = cv.blur(self.frame, (3,3))

            #converting to HSV
            hsv = cv.cvtColor(self.frame,cv.COLOR_BGR2HSV)

            # get info from track bar and appy to result
            hmin = cv.getTrackbarPos('hmin','result')
            hmax = cv.getTrackbarPos('hmax','result')
            smin = cv.getTrackbarPos('smin','result')
            smax = cv.getTrackbarPos('smax','result')
            vmin = cv.getTrackbarPos('vmin','result')
            vmax = cv.getTrackbarPos('vmax','result')

            # Normal masking algorithm
            lower_hsv = np.array([hmin, smin, vmin])
            upper_hsv = np.array([hmax, smax, vmax])

            mask_trackbar = cv.inRange(hsv,lower_hsv, upper_hsv)

            lower_hsv_orng = np.array([1,81,104])
            upper_hsv_orng = np.array([10,219,255])
            mask_orng = cv.inRange(hsv,lower_hsv_orng, upper_hsv_orng)

            lower_hsv_pink = np.array([170,14,150])
            upper_hsv_pink = np.array([179,220,255])
            mask_pink = cv.inRange(hsv,lower_hsv_pink, upper_hsv_pink)
                      
            track_x, track_y, mask3 = self.trackFilteredObject(mask_trackbar)
            #orng_x, orng_y, mask1 = self.trackFilteredObject(mask_orng)
            pink_x, pink_y, mask2 = self.trackFilteredObject(mask_pink)

       
            display = self.frame.copy()
            cv.circle(display, (track_x, track_y), 10, 255, -1) 
            #cv.circle(display, (orng_x, orng_y), 10, (0,0,255), -1) 
            cv.circle(display, (pink_x, pink_y), 10, (0,255,0), -1) 

            #cv.imshow('Thresh orange',mask1)
            cv.imshow('Thresh pink',mask2)
            cv.imshow('Thresh trackbar',mask3)
            cv.imshow('result', display)

            camera_vels = self.calc_end_velocities(pink_x, pink_y, 1)
            print camera_vels

            k = cv.waitKey(5) & 0xFF
            if k == 27:
                break

        self.capture.release()

        cv.destroyAllWindows()
        

if __name__=="__main__":
    v = Vision()
    v.run()
