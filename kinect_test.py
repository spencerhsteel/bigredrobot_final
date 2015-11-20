#import the necessary modules
import freenect
import cv2
import numpy as np

import time
import cPickle as pickle 
 
#function to get RGB image from kinect
def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array
 
#function to get depth image from kinect
def get_depth():
    array,_ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array
 
if __name__ == "__main__":

    recorded = []

    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # fourcc = cv2.cv.CV_FOURCC(*'XVID')
    # out = cv2.VideoWriter('output.avi',fourcc, 30.0, (640,480))

    timePrev = time.time() - .001
    while 1:
        print 1.0/(time.time() - timePrev)
        timePrev = time.time()

        #get a frame from RGB camera
        frame = get_video()
        #get a frame from depth sensor
        depth = get_depth()

        # Store
        # recorded.append((frame, depth))
        # out.write(frame)


        #display RGB image
        cv2.imshow('RGB image',frame)
        #display depth image
        cv2.imshow('Depth image',depth)
 
        # quit program when 'esc' key is pressed
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    # cap.release()
    # out.release()
    cv2.destroyAllWindows()

    # with open("kinect.pickle", "w") as f:
    #     pickle.dump(recorded, f)