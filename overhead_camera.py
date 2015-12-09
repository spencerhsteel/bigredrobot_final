import cv2 as cv
import visionlib as vl
import numpy as np
import rospy

USE_TRACKBARS = False
USE_DEPTHMASK = False
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
            h = int(box[1][0]) 
            w = int(box[1][1])
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
        cx = 328
        cy = 198
        x = 51
        y = 62
        w = 554
        h = 272

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
    side = 'NONE'
    while not rospy.is_shutdown():

        #_, self.frame = self.capture.read()
        rgb = camera.get_frame()
        #depth = camera.get_depth_frame()
        #z_data = depth          
        #create table mask
        if USE_DEPTHMASK:
            for v in range(0,640):
                for u in range(0,480):
                    if z_data[u][v] > 2000 or z_data[u][v] < 1500:
                        rgb[u][v] = (0,0,0)
        

        frame = cv.blur(rgb, (3,3))        
        #if FIND_ROI:
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

        gray = cv.cvtColor(display,cv.COLOR_BGR2GRAY)
        circles = cv.HoughCircles(gray,cv.cv.CV_HOUGH_GRADIENT,2,20,
                                    param1=100,param2=35,minRadius=5,maxRadius=15)
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv.circle(display,(i[0],i[1]),i[2],(0,255,0),2)


        lower_hsv_pink = np.array([140, 79, 78])
        upper_hsv_pink = np.array([179, 227, 255])
        mask_pink = cv.inRange(hsv,lower_hsv_pink, upper_hsv_pink)
        #pink_x, pink_y, mask_pink, _ = track_object_local(mask_pink, circles)
        pink_x, pink_y, mask_pink, _ = vl.track_object(mask_pink)        

        
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

        print side

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

def track_object_local(mask, locations):
    xy_coords = locations
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(5,5))
    morphed_mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    contours, hierarchy = cv.findContours(morphed_mask.copy(), cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
    cnts = sorted(contours, key = cv.contourArea, reverse = True)[:10]
    max_area = 0
    best_cnt = None
    area = -1
    tracked_x = -1
    tracked_y = -1
    for cnt in cnts:
      moment = cv.moments(cnt)
      area = moment['m00']
      x = int(moment['m10']/area)
      y = int(moment['m01']/area)
      print x,y 
      for location in locations:
        lx = location[0][0]
        ly = location[0][1]
        print lx,ly
        if (x < lx+5 and x > lx-5) and (y < ly+5 and y > ly-5):
            #it is also a circle
            tracked_x = x
            tracked_y = y
            print 'FOUND'
            break

    return tracked_x, tracked_y, morphed_mask, area

if __name__=="__main__":
    rospy.init_node('overhead_camera', anonymous=True)
    camera = vl.OverheadCamera()
    run(camera)
