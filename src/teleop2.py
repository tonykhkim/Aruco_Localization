#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cvbridge_tutorials')  # 'cvbridge_tutorials' is package name
import rospy
from std_msgs.msg import String,Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math

class cmd_pid():

    #global error
    #global angle
    #angle=None
    #error=None

    def __init__(self,Kp,Ki,Kd):
        self.vel_pub=rospy.Publisher("cmd_vel",Twist,queue_size=1)  # no function in publisher
        self.error_sub=rospy.Subscriber("error_info",Int32,self.callback,queue_size=1)
        #self.pose_sub=rospy.Subscriber("pose",Pose,self.callback,queue_size=1)
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.p_error=0.0
        self.i_error=0.0
        self.d_error=0.0
        #self.error=0.0
        
        
    def pid_control(self,err):
        print(err)
        self.d_error = err-self.p_error
        self.p_error = err
        self.i_error += err
        
        return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error
    
    def callback(self,data):
        #global angle
        
        self.error = data.data
        #self.d_error = error-self.p_error
        #self.p_error = error
        #self.i_error += error
        #angle = self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error
        #print(angle)
        #print(error)
        angular=self.pid_control(self.error)
        
        cmd = Twist()
        #cmd.linear.x = linear
        cmd.angular.z = -angular
        rospy.loginfo(cmd)
        
        try:
            self.vel_pub.publish(cmd)
        except KeyboardInterrupt:
            print("Shutting down")
"""   
    def drive(self):
        
        angular=self.pid_control(error)
        
        
        linear = 5
        
        
        # velocity
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = -angular
        
        try:
            self.vel_pub.publish(cmd)
        except KeyboardInterrupt:
            print("Shutting down")

        # far, fast / close, slow speed
        
"""  
        
if __name__=='__main__':
    rospy.init_node('cmd_pid', anonymous=True)
    cp = cmd_pid(0.5,0.0005,0.05)
    #angle=cp.pid_control(error)
    #cp.drive()
    try:
        rospy.spin() 
    except KeyboardInterrupt:
        print("Shutting down")
