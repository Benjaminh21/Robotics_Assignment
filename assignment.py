#!/usr/bin/env python

import rospy 
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from tf import transformations

#roslaunch uol_turtlebot_simulator maze1.launch

twist_pub_ = None
regions_ = {
    'right': 0,
    'centre': 0,
    'left': 0,
}

state_ = 2
current_state_ = {
    0: 'Turn Around',
    1: 'Turn Left',
    2: 'Forward'
}

def callback_laser(msg):
    #print len(msg.ranges)
    #print msg.ranges[360]

    global regions_
    regions_ = {
        'right': msg.ranges[0],
        'centre': msg.ranges[320],
        'left': msg.ranges[639],
    }

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

    distance = 0.5

    if regions['centre'] < distance:
        print "Wall ahead"
        if regions['left'] < distance and regions['right'] < distance:
            print "Wall both sides Turn around"
            state(0)
        else:
            print "Turn left"
            state(1)
    else:
        print "Forward"
        state(2)
        rospy.loginfo(regions)

#    twist_pub.publish(msg)




def turnAround():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0.6
    print "test"
    return msg

def turnLeft():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def forward():
    msg = Twist()
    msg.linear.x = 0.3
    print "other test"
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
        else:
            rospy.logerr('Unknown state!')
        
        twist_pub_.publish(msg)
        
        rate.sleep()
 

    #rospy.spin()

if __name__ == '__main__':
    main()


    
