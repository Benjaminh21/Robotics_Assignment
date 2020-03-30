#!/usr/bin/env python

import rospy 
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
import cv2
from cv_bridge import CvBridge, CvBridgeError
from cv2 import namedWindow, cvtColor, imshow
from cv2 import COLOR_BGR2Gray, waitKey
from cv2 import blur, Canny


#roslaunch uol_turtlebot_simulator maze1.launch

#Some code including the laser regions set-up was originally taken from 
#theconstructsim.com/wall-follower/algorithm/
#Alot has been changed but the original set up was from here.#

twist_pub_ = None
regions_ = {
    'right': 0,
    'right/centre': 0,
    'centre': 0,
    'left/centre': 0,
    'left': 0,
}

state_ = 2
current_state_ = {
    0: 'Turn Left',
    1: 'Turn Right',
    2: 'Forward',
}

def callback_laser(msg):
    #print len(msg.ranges)
    #print msg.ranges[360]

    global regions_
    regions_ = {
        'right': min(min(msg.ranges[0:39]), 10),
        'right/centre':min(min(msg.ranges[40:240]), 10),
        'centre': min(min(msg.ranges[241:398]), 10),
        'left/centre': min(min(msg.ranges[399:599]), 10),
        'left': min(min(msg.ranges[600:639]), 10),
    }

    #print len(msg.ranges)
    print regions_
    move()

def callback_image(data):
    namedWindow("Image Window")
    namedWindow("blur")
    namedWindow("canny")
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    gray_img = cvtColor(cv_image, COLOR_BGR2Gray)
    print mean(gray_img)
    img2 = Canny(gray_img, 10, 200)
    imshow("canny", im2)

    imshow("Image window", cv_image)
    waitKey(1)


def state(state):
    global state_
    global current_state_
    if state is not state_:
        print 'Current state - [%s] - %s' % (state, current_state_[state])
        state_ = state

def move():
    global regions_
    regions = regions_
    global twist_pub
    msg = Twist()
    linear_x = 0
    angular_z = 0

    distance = 0.6
    distance2 = 0.5
    distance3 = 0.3

    if regions['right'] < regions['left']:
        LRR = 1
    else:
        LRR = 2

    #LRR = np.random.randint(0,2)
    #print "LRR"
    #print LRR

    if regions['right'] < distance3:
        state(0)
    elif regions['right/centre'] < distance2:
        state(0)
    elif regions['left'] < distance3:
        state(1)
    elif regions['left/centre'] < distance2:
        state(1)
    elif regions['centre'] < distance:
        if LRR == 1:
            state(0)
        elif LRR == 2:
            state(1)
    else:
        print "Forward"
        state(2)
        #rospy.loginfo(regions)

#    twist_pub.publish(msg)



def turnLeft():
    time = 5
    msg = Twist()
    #msg.angular.z = pi*2/4/time
    msg.angular.z = 1
    return msg

def turnRight():
    msg = Twist()
    msg.angular.z = -1
    print "test"
    return msg

def forward():
    msg = Twist()
    msg.linear.x = 0.1
    #print "Forward"
    return msg
        

def main():
    global twist_pub_
    global state_

    rospy.init_node("Explorer")
		
    cv2.startWindowThread()    
    bridge = CvBridge()

    #Subscribers
    laser_sub = rospy.Subscriber("/scan", LaserScan, callback_laser)
    image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, callback_image)

    #Publishers
    twist_pub_ = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = turnLeft()
            rospy.logerr("left")
        elif state_ == 1:
            msg = turnRight()
            rospy.logerr("right")
        elif state_ == 2:
            msg = forward()
        else:
            rospy.logerr('Unknown state!')
            
        
        twist_pub_.publish(msg)
        
        rate.sleep()
 

    #rospy.spin()

if __name__ == '__main__':
    main()


    
