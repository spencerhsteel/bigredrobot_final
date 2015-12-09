import cv2 as cv
import visionlib as vl
import numpy as np
import rospy

USE_TRACKBARS = False
USE_DEPTHMASK = False
FIND_ROI = True

def run(camera):
    
    if FIND_ROI:
        box = None
        while not rospy.is_shutdown():
            
            while box is None:
                rgb = camera.get_frame()
                frame = cv.blur(rgb, (4,4))
                # While loop allows us to 
                setupDisplay = frame.copy()
                field, box = find_ROI(frame)
            cx = int(box[0][0])
            cy = int(box[0][1])
            w = int(box[1][0]) 
            h = int(box[1][1])
            x = cx - w/2
            y = cy - h/2
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
        
        if FIND_ROI:
            copy = frame.copy()        
            mask = np.zeros(copy.shape[:2],np.uint8)
            mask[y:y+h,x:x+w] = 255
            frame = cv.bitwise_and(copy,copy,mask = mask)

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
            track_x, track_y, mask3, _ = vl.track_object(mask_trackbar)
            cv.circle(display, (track_x, track_y), 5, 255, -1) 
            cv.imshow('Thresh trackbar',mask3)

        lower_hsv_pink = np.array([140, 79, 78])
        upper_hsv_pink = np.array([179, 227, 255])
        mask_pink = cv.inRange(hsv,lower_hsv_pink, upper_hsv_pink)
        pink_x, pink_y, mask_pink, _ = vl.track_object(mask_pink)        

        cv.circle(display, (pink_x, pink_y), 5, (0,0,255), -1) 

        cv.imshow('result', display)
        cv.imshow('Pink', mask_pink)

        #What side of field?
        if pink_x >= cx:
            print 'right'
        else:
            print 'left'

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

if __name__=="__main__":
    rospy.init_node('overhead_camera', anonymous=True)
    camera = vl.OverheadCamera()
    run(camera)
