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
#from cv2 import COLOR_BGR2Gray, waitKey
from cv2 import blur, Canny


#roslaunch uol_turtlebot_simulator maze1.launch

#Some code including the laser regions set-up was originally taken from 
#theconstructsim.com/wall-follower/algorithm/
#Alot has been changed but the original set up was from here.#

twist_pub_ = None
regions_ = {                #Laser scan regions are split into 5 shown here#
    'right': 0,             #This allows for the robot to detect obstacles in specific regions#
    'right/centre': 0,
    'centre': 0,
    'left/centre': 0,
    'left': 0,
}

state_ = 2
current_state_ = {          #If the robot detects an obstacle, it will change state#
    0: 'Turn Left',         #If a wall is to the right it will change to turn left#
    1: 'Turn Right',
    2: 'Forward',           #Moving forward is the default state#
}

def callback_laser(msg):
    #print len(msg.ranges)
    #print msg.ranges[360]

    global regions_
    regions_ = {
        'right': min(min(msg.ranges[0:39]), 10),            #Here I split the laser scan ranges#
        'right/centre':min(min(msg.ranges[40:240]), 10),    #The lowest value from the range will be returned#
        'centre': min(min(msg.ranges[241:398]), 10),        #So out if the 100 scan lines on left/centre the one with the lowest value is returned#
        'left/centre': min(min(msg.ranges[399:599]), 10),   #Lowest value means the one with the closest object#
        'left': min(min(msg.ranges[600:639]), 10),
    }

    #print len(msg.ranges)
    print regions_
    move()

# def callback_image(data):
#     try:
#         cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
#     except CvBridgeError as e:
#         print(e)

#     cv2.imshow("Image Window", cv_image)
#     cv2.waitkey(1)


def state(state):
    global state_
    global current_state_
    if state is not state_:
        print 'Current state - [%s] - %s' % (state, current_state_[state])      #If a requirement is met e.g. obstacle in the way#
        state_ = state                                                          #The state is changed and a print message is displayed to tell the user#

def move():
    global regions_
    regions = regions_
    global twist_pub
    msg = Twist()   #Twist message is created#
    linear_x = 0    #Forward and twist speed is set to 0 by default#
    angular_z = 0

    distance = 0.6  #These distances are from an object or wall#
    distance2 = 0.5 #0.5 means 0.5 metres from ibstac
    distance3 = 0.3

    if regions['right'] < regions['left']:
        LRR = 1                             #LRR is used to determine whether to turn left or right#
    else:                                   #If there is more space on the left it will turn left and vise versa#
        LRR = 2

    if regions['right'] < distance3:            #If there is a wall less than 0.3 metres to the right, the robot will change state and turn left slightly#
        state(0)
    elif regions['right/centre'] < distance2:   #If there is a wall less than 0.5 metres in front and to the right, the robot will turn left slightly#
        state(0)
    elif regions['left'] < distance3:           #If there is a wall less than 0.6 metres to the left, the robot will change to state 1 and turn right#
        state(1)
    elif regions['left/centre'] < distance2:
        state(1)
    elif regions['centre'] < distance:          #If there is a wall directly ahead, the robot will decide whether to turn left or right#
        if LRR == 1:
            state(0)
        elif LRR == 2:
            state(1)
    else:
        print "Forward"
        state(2)                                #Default state for the robot is to move forward#
        #rospy.loginfo(regions)

#    twist_pub.publish(msg)



def turnLeft():
    time = 5
    msg = Twist()
    #msg.angular.z = pi*2/4/time
    msg.angular.z = 1               #If the robot needs to turn left the angular momentem is changed to 1#
    return msg                      #Twist message is returned#                           
                                    #This is repreated for turning right and moving forward#
def turnRight():
    msg = Twist()
    msg.angular.z = -1
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
		
    bridge = CvBridge()

    #Subscribers
    laser_sub = rospy.Subscriber("/scan", LaserScan, callback_laser)
    #image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, callback_image)

    #Publishers
    twist_pub_ = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:            #If the current state = 0, then the robot needs to turn left and the turnLeft function is called#
            msg = turnLeft()
            rospy.logerr("left")
        elif state_ == 1:          #If the current state = 1, then the robot needs to turn right and the turnRight function is called#
            msg = turnRight()
            rospy.logerr("right")
        elif state_ == 2:
            msg = forward()
        else:
            rospy.logerr('Unknown state!')  #If the state is unknown an error message is displayed to the user#
            
        
        twist_pub_.publish(msg)     #Twist message is published to tell the robot what movement to carry out#
        
        rate.sleep()                #The program sleeps to allow the robot to carry out movement before checking for a new state#

        #cv2.destroyAllWindows()
 

    #rospy.spin()

if __name__ == '__main__':
    main()


    
