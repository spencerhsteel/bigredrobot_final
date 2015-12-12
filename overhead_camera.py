#!/usr/bin/env python
import cv2 as cv
import lib.visionlib as vl
import numpy as np
import rospy
from bigredrobot_final.msg import *

USE_TRACKBARS = False
USE_DEPTHMASK = True
FIND_ROI = False

def run(camera):
    
    if FIND_ROI:
        box = None
        while not rospy.is_shutdown():
            #h552 w274 cx 331 cy 199
            while box is None:
                rgb = camera.get_frame()
                frame = cv.blur(rgb, (4,4))
                # While loop allows us to 
                setupDisplay = frame.copy()
                field, box = find_ROI(frame)
                k = cv.waitKey(5) & 0xFF
                if k == 27:
                    break
            cx = int(box[0][0])
            cy = int(box[0][1])
            h =  int(box[1][1])
            w =  int(box[1][0])
            x = cx - w/2
            y = cy - h/2
            print cx,cy,x,y,w,h
            cv.line(setupDisplay, (cx, cy+h/2), (cx, cy-h/2), (0,0,255), 4)
            cv.rectangle(setupDisplay, (x,y), (x+w,y+h), (255,0,0), 2)
            cv.drawContours(setupDisplay, [field], -1, (0, 255, 0), 3)
            cv.imshow('ROI', setupDisplay)
            k = cv.waitKey(5) & 0xFF
            # 'ENTER' to cotinue (happy with ROI)
            if k == 10:
                cv.destroyAllWindows()
                break
            # 'ESC' to quit and try again
            elif k == 27:
                cv.destroyAllWindows()
                return
            
    else:
        cx = 333
        cy = 181
        x = 50
        y = 44
        w = 567
        h = 275

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
    side = 'NONE'
    while not rospy.is_shutdown():

        rgb = camera.get_frame()
        depth = camera.get_depth_frame()

        frame = cv.blur(rgb, (3,3))          
        mask = np.zeros(frame.shape[:2],np.uint8)
        mask[y:y+h,x:x+w] = 255
        frame = cv.bitwise_and(frame,frame,mask = mask)

        #create table mask
        if USE_DEPTHMASK:
            _, depth_thresh = cv.threshold(depth, 5, 255, cv.THRESH_BINARY)
            d = np.zeros_like(depth_thresh)
            y_offset = 20
            x_offset_left = 20
            x_offset_right = 12
            mid = frame.shape[0]/2
            right_lim = frame.shape[0]/1.5
            d[y_offset:] = depth_thresh[:-y_offset]
            d[:,x_offset_left:mid] = d[:,:mid-x_offset_left]
            d[:,right_lim-x_offset_right:-x_offset_right] = d[:,right_lim:]
            d = cv.dilate(d, np.ones((15,15)))
            d = cv.erode(d, np.ones((15,15)))
            frame = cv.dilate(frame, np.ones((5,5)))
            frame = cv.erode(frame, np.ones((5,5)))
            frame = cv.bitwise_and(frame,frame,mask = d)

        #converting to HSV
        hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
        track_x = -1
        track_y = -1

        display = frame.copy()

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
            track_x, track_y, mask3, _ = track_ball(mask_trackbar)
            cv.circle(display, (track_x, track_y), 5, 255, -1) 
            cv.imshow('Thresh trackbar',mask3)

        gray = cv.cvtColor(display,cv.COLOR_BGR2GRAY)

        lower_hsv_pink = np.array([140, 20, 100])
        upper_hsv_pink = np.array([179, 250, 255])
        mask_pink = cv.inRange(hsv,lower_hsv_pink, upper_hsv_pink)
        pink_x, pink_y, mask_pink, _ = track_ball(mask_pink)        

        
        cv.circle(display, (pink_x, pink_y), 5, (0,0,255), -1) 
        
        cv.imshow('result', display)
        cv.imshow('Pink', mask_pink)

        #What side of field?
        if pink_x == -1:
            #it is under the arm
            pass
        elif pink_x >= cx:
            side = 'right'
        else:
            side =  'left'

        #publish data
        state = OverheadCamera()
        state.center_x = cx
        state.center_y = cy
        state.width = w
        state.height = h
        state.ball_x = pink_x
        state.ball_y = pink_y
        pub.publish(state)
        rate.sleep()

        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break

    cv.destroyAllWindows()

def find_ROI(frame):
    #From http://www.pyimagesearch.com/2014/04/21/building-pokedex-python-finding-game-boy-screen-step-4-6/
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gray = cv.bilateralFilter(gray, 11, 20, 17)
    edged = cv.Canny(gray, 30, 200)
    cv.imshow('edged', edged)
    (cnts, _) = cv.findContours(edged.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key = cv.contourArea, reverse = True)[:10]
    fieldCnt = None
    fieldBox = None
    for c in cnts:
        peri = cv.arcLength(c, True)
        approx = cv.approxPolyDP(c, 0.02 * peri, True)
        moment = cv.moments(c)
        area = moment['m00']

        if len(approx) == 4 and area > 100000:
            fieldCnt = approx
            fieldBox = cv.minAreaRect(c)
            break
    return fieldCnt, fieldBox

def track_ball(mask):
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(10,10))
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

if __name__=="__main__":
    pub = rospy.Publisher('/bigredrobot/overhead_camera', OverheadCamera, queue_size = 10)
    rospy.init_node('overhead_camera', anonymous=True)
        
    camera = vl.OverheadCamera()
    run(camera)

