import cv2 as cv
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError


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

        cv.namedWindow("Baxter Image", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.callback)

    def callback(self,data):
       try:
         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
       except CvBridgeError, e:
         print e
       (rows,cols,channels) = cv_image.shape
       if cols > 60 and rows > 60 :
       cv.circle(cv_image, (50,50), 10, 255)
   
       cv.imshow("Image window", cv_image)
       cv.waitKey(3)
   
       try:
         self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
       except CvBridgeError, e:
         print e

    def on_trackbar(self,value):
        return 0

    def drawCrosshairs(self,x,y):
        x=int(x)
        y=int(y)
        cv.circle(self.frame,(x,y),20,(0,255,0),2)
        if (y-25)>0:
            cv.line(self.frame, (x,y), (x,y-25), (0, 255, 0), 2)
        else:
            cv.line(self.frame, (x,y), (x,0), (0, 255, 0), 2);
        if (y+25)<self.FRAME_HEIGHT:
            cv.line(self.frame, (x,y), (x,y + 25), (0, 255, 0), 2)
        else:
            cv.line(self.frame, (x,y), (x,self.FRAME_HEIGHT), (0, 255, 0), 2)
        if (x-25)>0:
            cv.line(self.frame, (x,y), (x-25,y), (0, 255, 0), 2)
        else:
            cv.line(self.frame, (x,y), (0,y), (0, 255, 0), 2)
        if (x+25)<self.FRAME_WIDTH:
            cv.line(self.frame, (x,y), (x+25,y), (0, 255, 0), 2)
        else:
            cv.line(self.frame, (x,y), (self.FRAME_WIDTH,y), (0, 255, 0), 2)

    def trackFilteredObject(self,temp):
        _, contours, hierarchy = cv.findContours(temp.copy(), cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
        refArea = 0
        indexFound = -1
        if hierarchy is not None:
            if hierarchy.shape[1] > 0:
                numObjects = hierarchy.shape[1]
                if numObjects < self.MAX_NUM_OBJECTS:
                    index = 0
                    while index >= 0:
                        moment = cv.moments(contours[index])
                        area2 = moment['m00']
                        if (area2>self.MIN_OBJECT_AREA) and (area2<self.MAX_OBJECT_AREA) and (area2>refArea):
                            self.objectx = moment['m10']/area2
                            self.objecty = moment['m01']/area2
                            self.objectFound = True
                            refArea = area2
                            indexFound = index
                        else:
                            self.objectFound = False
                            self.objectx = -1
                            self.objecty = -1
                        index = hierarchy[0][index][0]		
                    if self.objectFound == True:
                        #angle = self.horizontalAngle(self.yx)
                        #self.balld = self.calculateBallDistance(refArea)
                        #self.balla = angle
                        #self.ballx = self.yx
                        #self.bally = self.yy
                        self.area = refArea

    def run(self):

        # Creating a window for later use
        cv.namedWindow('result')

        # Creating track bar
        cv.createTrackbar('hmin', 'result',0,179,self.on_trackbar)
        cv.createTrackbar('hmax', 'result',179,179,self.on_trackbar)
        cv.createTrackbar('smin', 'result',0,255,self.on_trackbar)
        cv.createTrackbar('smax', 'result',255,255,self.on_trackbar)
        cv.createTrackbar('vmin', 'result',0,255,self.on_trackbar)
        cv.createTrackbar('vmax', 'result',255,255,self.on_trackbar)

        while(1):

            _, self.frame = self.capture.read()

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

            mask = cv.inRange(hsv,lower_hsv, upper_hsv)

            result = cv.bitwise_and(self.frame,self.frame,mask = mask)
            
            self.trackFilteredObject(mask)

            self.drawCrosshairs(self.objectx, self.objecty)

            cv.imshow('Thresh',mask)
            cv.imshow('result',self.frame)

            k = cv.waitKey(5) & 0xFF
            if k == 27:
                break

        cap.release()

        cv.destroyAllWindows()

if __name__=="__main__":
    v = Vision()
    v.run()
