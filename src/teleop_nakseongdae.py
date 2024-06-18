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
        self.pose_sub=rospy.Subscriber("pose2",Pose,self.callback,queue_size=1)
    
    def callback(self,pose):
        # pose
        # math.atan2() method returns the arc tangent of y/x, in float radians value. where x and y are the coordinates of a point (x,y). The returned value is between PI and -PI.
        angular = math.atan2(pose.position.x, pose.position.z)
        linear = 0.2756*math.sqrt(pose.position.x ** 2 + pose.position.z ** 2)   #need to tune the parameter
        # angular = 0.05*math.atan2(pose.position.x, pose.position.y)
        # linear = 0.05*math.sqrt(pose.position.x ** 2 + pose.position.y ** 2)
        print('angle : ',angular)
        print('distance : ',linear)
        
        #print('x orientation : ',pose.orientation.x)
        #print('y orientation : ',pose.orientation.y)
        #print('z orientation : ',pose.orientation.z)
            
    
        
if __name__=='__main__':
    rospy.init_node('cmd_pose', anonymous=True)
    ic=cmd_pose()
    try:
        rospy.spin() 
    except KeyboardInterrupt:
        print("Shutting down")
