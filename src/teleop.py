#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cvbridge_tutorials')  # 'cvbridge_tutorials' is package name
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
import math

class cmd_pose:

    def __init__(self):
        self.vel_pub=rospy.Publisher("cmd_vel",Twist,queue_size=1)  # no function in publisher
        self.pose_sub=rospy.Subscriber("pose",Pose,self.callback1,queue_size=1)
        self.msg_sub=rospy.Subscriber("check_arrive",Int32,self.callback2,queue_size=1)
        
        self.flag=0
    
    def callback1(self,pose):
        # pose
        # math.atan2() method returns the arc tangent of y/x, in float radians value. where x and y are the coordinates of a point (x,y). The returned value is between PI and -PI.
        angular = math.atan2(pose.position.x, pose.position.z)
        linear = 0.48*math.sqrt(pose.position.x ** 2 + pose.position.z ** 2)   #need to tune the parameter
        # angular = 0.05*math.atan2(pose.position.x, pose.position.y)
        # linear = 0.05*math.sqrt(pose.position.x ** 2 + pose.position.y ** 2)
        print('angle : ',angular)
        print('distance : ',linear)
        
        #print('x orientation : ',pose.orientation.x)
        #print('y orientation : ',pose.orientation.y)
        #print('z orientation : ',pose.orientation.z)

        # # speed control
        if linear<0.7:
            linear=0.0
        else:
            linear=linear*0.1
        ##elif linear>0.78:
            ##linear=linear*0.1
        
        if angular<-1.0:
            angular=angular*0.2
        elif angular>1.0:
            angular=angular*0.2

        # velocity
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = -angular
        
        print('linear : ',linear)
        print('angular : ',angular)
        print('-------------------------------')
        
        ##try:
            ##self.vel_pub.publish(cmd)
        ##except KeyboardInterrupt:
            ##print("Shutting down")
            
        if self.flag == 1:
            self.vel_pub.publish(cmd)
            
    def callback2(self,data):
        self.flag=data.data
        rospy.loginfo(self.flag)
        
        ##if self.flag:
            ##callback1

        # far, fast / close, slow speed

##class check_flag:
    ##def __init__(self):
        ##self.msg_sub=rospy.Subscriber("navigation_to_markerdetection",Int32,self.callback2,queue_size=1)
    
    ##def callback2(self,data):
        ##self.flag=msg.data
        
##def callback(msg):
    
    ##if msg.data == 1:
        ##print("flag is arrive")    
    #flag=msg.data
        
if __name__=='__main__':
    rospy.init_node('cmd_pose', anonymous=True)
    #cf=check_flag()
    ##rospy.init_node('check_flag')
    ##msg_sub=rospy.Subscriber("navigation_to_markerdetection",Int32,callback)
    ic=cmd_pose()
    #if ic.flag == 1:
        #vel_pub.publish(cmd)
        ##ic = cmd_pose()
    #ic = cmd_pose()
    try:
        rospy.spin() 
    except KeyboardInterrupt:
        print("Shutting down")
