import cv2 as cv
import visionlib as vl
import numpy as np
import rospy

USE_TRACKBARS = True
USE_DEPTHMASK = True

def run(camera):
    cv.namedWindow('result')
    # Creating track bar
    if USE_TRACKBARS:
        callback = lambda x: 0
        cv.createTrackbar('hmin', 'result', 0, 179, callback)
        cv.createTrackbar('hmax', 'result',179,179, callback)
        cv.createTrackbar('smin', 'result',0,255, callback)
        cv.createTrackbar('smax', 'result',255,255, callback)
        cv.createTrackbar('vmin', 'result',0,255, callback)
        cv.createTrackbar('vmax', 'result',255,255, callback)
    rate = rospy.Rate(10)
    z_data = None
    while not rospy.is_shutdown():

        #_, self.frame = self.capture.read()
        rgb = camera.get_frame()
        depth = camera.get_depth_frame()
        z_data = depth        
                
        #create table mask
        if USE_DEPTHMASK:
            for v in range(0,640):
                for u in range(0,480):
                    if z_data[u][v] > 2000 or z_data[u][v] < 1500:
                        rgb[u][v] = (0,0,0)
        

        frame = cv.blur(rgb, (3,3))

        #converting to HSV
        hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
        track_x = -1
        track_y = -1

        if USE_TRACKBARS:
            hmin = cv.getTrackbarPos('hmin','result')
            hmax = cv.getTrackbarPos('hmax','result')
            smin = cv.getTrackbarPos('smin','result')
            smax = cv.getTrackbarPos('smax','result')
            vmin = cv.getTrackbarPos('vmin','result')
            vmax = cv.getTrackbarPos('vmax','result')
            lower_hsv = np.array([hmin, smin, vmin])
            upper_hsv = np.array([hmax, smax, vmax])
            mask_trackbar = cv.inRange(hsv,lower_hsv, upper_hsv)
            track_x, track_y, mask3 = vl.track_object(mask_trackbar)
            cv.imshow('Thresh trackbar',mask3)

        '''lower_hsv_pink1 = np.array([160,14,10])
        upper_hsv_pink1 = np.array([179,220,255])
        mask_pink1 = cv.inRange(hsv,lower_hsv_pink1, upper_hsv_pink1)

        lower_hsv_pink2 = np.array([0, 14, 10])
        upper_hsv_pink2 = np.array([8, 220, 255])
        mask_pink2 = cv.inRange(hsv, lower_hsv_pink2, upper_hsv_pink2)

        mask_pink = cv.bitwise_or(mask_pink1, mask_pink2)
        '''
        #pink_x, pink_y, mask2 = vl.track_object(mask_pink)

        display = frame.copy()
        cv.circle(display, (track_x, track_y), 5, 255, -1) 

        #cv.imshow('Thresh pink',mask2)
        cv.imshow('result', display)

        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break

    cv.destroyAllWindows()


if __name__=="__main__":
    rospy.init_node('overhead_camera', anonymous=True)
    camera = vl.OverheadCamera()
    run(camera)
