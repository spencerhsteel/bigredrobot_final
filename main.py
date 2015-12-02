import cv2 as cv
import visionlib as vl
import rospy
import numpy as np

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

from std_msgs.msg import (
    UInt16,
    Header
)

from geometry_msgs.msg import Vector3Stamped, Vector3

import tf

class Baxter:
    def __init__(self, arm):
        rospy.logwarn("Initializing Baxter")

        self.arm = baxter_interface.limb.Limb(arm)
        self.kinematics = baxter_kinematics(arm)

        # control parameters
        self.pub_rate = 500.0  # Hz

        self.interface = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self.interface.state().enabled
        self.interface.enable()
        
        # set joint state publishing to 500Hz
        self.rate_publisher = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self.rate_publisher.publish(self.pub_rate)


def run(camera):
   
    # Creating a window for later use
    cv.namedWindow('result')

    # Creating track bar
    callback = lambda x: 0
    cv.createTrackbar('hmin', 'result', 0, 179, callback)
    cv.createTrackbar('hmax', 'result',179,179, callback)
    cv.createTrackbar('smin', 'result',0,255, callback)
    cv.createTrackbar('smax', 'result',255,255, callback)
    cv.createTrackbar('vmin', 'result',0,255, callback)
    cv.createTrackbar('vmax', 'result',255,255, callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        #_, self.frame = self.capture.read()
        frame = cv.blur(camera.get_frame(), (3,3))

        #converting to HSV
        hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)

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
                  
        track_x, track_y, mask3 = vl.track_object(mask_trackbar)
        #orng_x, orng_y, mask1 = vl.track_object(mask_orng)
        pink_x, pink_y, mask2 = vl.track_object(mask_pink)

        display = frame.copy()
        cv.circle(display, (track_x, track_y), 10, 255, -1) 
        #cv.circle(display, (orng_x, orng_y), 10, (0,0,255), -1) 
        cv.circle(display, (pink_x, pink_y), 10, (0,255,0), -1) 

        #cv.imshow('Thresh orange',mask1)
        cv.imshow('Thresh pink',mask2)
        cv.imshow('Thresh trackbar',mask3)
        cv.imshow('result', display)

        visual_servo(camera, pink_x, pink_y)
        rate.sleep()

        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break

    camera.capture.release()

    cv.destroyAllWindows()

def transform_test():
    tl = tf.TransformListener()
    
    vect = Vector3Stamped()
    vect.header.frame_id = '/right_hand_camera'
    vect.header.stamp = rospy.Time()
    vect.vector = Vector3(*[1,2,3])
    
    vectpub = rospy.Publisher('vector', Vector3Stamped)
    vecttranspub = rospy.Publisher('vectortrans', Vector3Stamped)

    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = tl.lookupTransform('/right_hand_camera', '/base', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            trans_vect = tl.transformVector3('/base', vect)
            print vect.header.frame_id, vect.vector
            print trans_vect.header.frame_id, trans_vect.vector
            vectpub.publish(vect)
            vecttranspub.publish(trans_vect)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()


def visual_servo(camera, u, v):
    tl = tf.TransformListener()
    
    xi = camera.calc_end_velocities(u, v, Z=1)
    print xi

    vect = Vector3Stamped()
    vect.header.frame_id = '/right_hand_camera'
    vect.header.stamp = rospy.Time()
    vect.vector = Vector3(*xi[0:3])

    while not tl.canTransform('/base', vect.header.frame_id, vect.header.stamp):
        try:
            trans_vect = tl.transformVector3('/base', vect)
            print vect.header.frame_id, vect.vector
            print trans_vect.header.frame_id, trans_vect.vector

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


if __name__=="__main__":
    rospy.init_node('main', anonymous=True)
    baxter = Baxter(arm='right')
    #transform_test()
    camera = vl.BaxterCamera(arm='right')
    run(camera)
