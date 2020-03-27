#!/usr/bin/env python

import rospy 
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
#from tf import transformations

#roslaunch uol_turtlebot_simulator maze1.launch

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
    0: 'Turn Around',
    1: 'Turn Left',
    2: 'Forward',
    3: 'Right'
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

    if regions['right'] > distance3:
        state(1)
    elif regions['centre'] < distance and regions['left'] > distance2:
        current_state = "State 1 left"
        state(1)
    else:
        print "Forward"
        state(2)
        #rospy.loginfo(regions)

#    twist_pub.publish(msg)




def turnAround():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 15
    print "Turning Around"
    return msg

def turnLeft():
    time = 5
    msg = Twist()
    #msg.angular.z = pi*2/4/time
    msg.angular.z = 0.3
    return msg

def turnRight():
    msg = Twist()
    msg.angular.z = -15
    return msg

def forward():
    msg = Twist()
    msg.linear.x = 0.1
    #print "Forward"
    return msg
        

def main():
    global twist_pub_

    rospy.init_node("Explorer")
		
    #Subscribers
    #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, callback_laser,queue_size=1)
    laser_sub = rospy.Subscriber("/scan", LaserScan, callback_laser)

    #Publishers
    twist_pub_ = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = turnAround()
        elif state_ == 1:
            msg = turnLeft()
            rospy.logerr("left")
        elif state_ == 2:
            msg = forward()
        elif state == 3:
            msg = turnRight()
        else:
            rospy.logerr('Unknown state!')
            
        
        twist_pub_.publish(msg)
        
        rate.sleep()
 

    #rospy.spin()

if __name__ == '__main__':
    main()


    
