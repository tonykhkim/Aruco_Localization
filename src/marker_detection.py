#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cvbridge_tutorials')  # 'cvbridge_tutorials' is package name
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2.aruco as aruco
import yaml
import os
from geometry_msgs.msg import Pose,Twist
import math
from scipy.spatial.transform import Rotation

##image_converter node receives(subscribe) the image_raw topic and emits(publish) a pose topic.

class image_converter:

  def __init__(self):
    self.pose_pub = rospy.Publisher("pose",Pose,queue_size = 1)  # publisher's topic name can be anything
    self.image_pub = rospy.Publisher("image", Image, queue_size=1)
    self.bridge = CvBridge()   #cvbridge --> help to link with opencv importing ros image 
    #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback,queue_size = 1)  
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback,queue_size = 1)  # '/usb_cam/image_raw' is usb_cam's topic name
    ##self.twist_pub=rospy.Publisher("twist",Twist,queue_size=1)
 
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
      cv_image1 = cv2.resize(cv_image,dsize=(10,10),interpolation=cv2.INTER_AREA)   #dsize: output image size
      # cv_image=cv2.medianBlur(cv_image,31) # odd,up 1
      #print('cv_image : ',cv_image)
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Change grayscale

      gray1 = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2GRAY)
      pixelsum=cv2.sumElems(gray1)    #sumElems(src): Calculate the sum of the elements for each channel in the array
      #print('sum : ',pixelsum)
      ##aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers  (5x5: number of rectangles inside the marker  250: id range 0~249 )
      ##parameters = aruco.DetectorParameters_create()  # Marker detection parameters
      aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_5X5_250)  #indicates the type of markers that will be searched
      parameters=aruco.DetectorParameters()    #marker detection parameters
      corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
      #corners: vector of detected marker corners. For N detected markers, the dimensions of this array is Nx4. The order of the corners is clockwise.
      #ids: vector of identifiers of the detected markers. The identifier is of type int. For N detected markers, the size of ids is also N.
      #rejected_img_points: contains the imgPoints of those squares whose inner code has not a corrected codification. Useful for debugging purpose
      p = Pose()
      #a = Twist()
      #if np.all(ids is None):  # If there are markers not found by detector
        #print('Marker undetected')
      if np.all(ids is not None):   # If there are markers found by detector
        #print('Marker detected')
        ##print('marker id : ',ids)      # print marker ID
        for i in range(0, len(ids)): # Iterate in markers
          rvec, tvec,markerPoints= aruco.estimatePoseSingleMarkers(corners[i], 0.1, matrix_coefficients, distortion_coefficients)  # markerLength width 0.1m    #rvec: rotation vector  #tvec: translation vector
          (rvec - tvec).any()  # get rid of that nasty numpy value array error
          # inversePerspective
          R, _ = cv2.Rodrigues(rvec) # converts rotation vector to rotation matrix using Rodrigues transformation   #Since rvec is Rodrigues' expression for rotation transformation, Opencv's Rodrigues() function should be used to obtain the actual rotation transformation matrix R.
          ##print("R: ",R)
          #R -> 3x3 matrix    R:  [[ 0.94088787  0.14640532 -0.30544313]
                               #   [ 0.10533923 -0.98352108 -0.14693509]
                               #   [-0.32192183  0.1060743  -0.94080528]] 
          r1=R[0]  #r1 -> 1x3 vector  #first row of Rotation transformation matrix R
          r2=R[1]  #r2 -> 1x3 vector  #second row of Rotation transformation matrix R
          r3=R[2]  #r3 -> 1x3 vector  #third row of Rotation transformation matrix R
          #R = np.matrix(R).T  # transposition   #get inverse matrix of above rotation matrix
          #print("before reshape tvec: ",tvec)   #tvec : [[[0.10289224 0.00963378 1.16447817]]]
          tvec1=np.reshape(tvec,(3,1))       #tvec1 -> 3x1 matrix
          #print("after reshape tvec: ",tvec1)    #tvec1 :   [[0.10289224]
                                                 #            [0.00963378]
                                                 #            [1.16447817]]
          t1=tvec1[0]            
          t2=tvec1[1]
          t3=tvec1[2]
          a1=np.append(r1,t1)  # add list
          a2=np.append(r2,t2)
          a3=np.append(r3,t3)
          a4=np.array([0,0,0,1])
          t=np.r_[[a1],[a2],[a3],[a4]]  # make 4x4 Homogeneous transformation matrix t(=rotation matrix + translation vector)
          #np.r_[[a1],[a2],[a3],[a4]] : merge 4 arrays from top to bottom
          #print('t : ',t)  
          ##tt=np.linalg.inv(t)  # get inverse Homogeneous transformation matrix
          #np.linalg: used to compute matrices
          #print('tt :',tt)  
          ##ct=np.array([tt[0][3],tt[1][3],tt[2][3]])  # camera pose
          #print('ct : ',ct)
          invRvec, _ = cv2.Rodrigues(R) # rotation's transposition = rotation's inverse
          aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)  # Draw a square around the markers
          ##aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)  # Draw Axis, axis length 0.05m
          cv2.drawFrameAxes(cv_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)
         
          cv2.imshow("Image window", cv_image)
          # pose publisher
          #rate = rospy.Rate(1) # Hz
          #while not rospy.is_shutdown():
          ##r=Rotation.from_matrix(R)
          ##quat=r.as_quat()
          
          #Quaternion format
          ##transform_rotation_x=quat[0]
          ##transform_rotation_y=quat[1]
          ##transform_rotation_z=quat[2]
          ##transform_rotation_w=quat[3]
          
          #Euler angle format in radians
          ##roll_x,pitch_y,yaw_z=euler_from_quaternion(transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w)
          
          ##roll_x=math.degrees(roll_x)
          ##pitch_y=math.degrees(pitch_y)
          ##yaw_z=math.degrees(yaw_z)
          
          ##a.linear.x=(transform_rotation_x)
          ##a.linear.y=(transform_rotation_y)
          ##a.linear.z=(transform_rotation_z)
          ##a.angular.x=(roll_x)
          ##a.angular.y=(pitch_y)
          ##a.angular.z=(yaw_z)
          ##rospy.loginfo(a)
          ##pub.publish(a)
          ##rate.sleep()
          
          
          
          # p.position.x = ct[0]
          # print(p.position.x)
          # p.position.y = ct[1]
          # p.position.z = ct[2]
          p.position.x = t1
          p.position.y = t2
          p.position.z = t3
          # Make sure the quaternion is valid and normalized
          #p.orientation.x = 0.0
          #p.orientation.y = 0.0
          #p.orientation.z = 0.0
          #p.orientation.w = 1.0

          #self.pub.publish(p)
          
          #rate.sleep()

    except CvBridgeError as e:
      print(e)
    
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)  # imshow & waitKey is essential. if waitKey doesn't exist, imshow turns off immediately

    try:
      self.pose_pub.publish(p) # After working with opencv image, convert to ros image again and publish.
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8")) # After working with opencv image, convert to ros image again and publish.
      ##self.twist_pub.publish(a)
      
    except CvBridgeError as e:
      print(e)
  
  # def pose(self,ct):
  #   try:
  #     p = Pose()
  #     p.position.x = ct[0]
  #     print(p.position.x)
  #     p.position.y = ct[1]
  #     p.position.z = ct[2]
  #     # Make sure the quaternion is valid and normalized
  #     p.orientation.x = 0.0
  #     p.orientation.y = 0.0
  #     p.orientation.z = 0.0
  #     p.orientation.w = 1.0
  #     self.pub.publish(p)
  #   except KeyboardInterrupt:
  #     print("Shutting down")
  #     cv2.destroyAllWindows()



def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

  
def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)
      
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)
      
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)
      
  return roll_x, pitch_y, yaw_z # in radians


if __name__ == '__main__':
  # yaml file load(matrix_coefficients, distortion_coefficients)
  with open('/home/unicon4/catkin_ws/src/ArUco_marker_detection/aruco/src/image_web/ost.yaml') as f: # ost.yaml : calibration again
    data=f.read()  # read loaded file
    vegetables = yaml.load(data,Loader=yaml.FullLoader)
    k=vegetables['K']    #camera matrix    #3x3
    d=vegetables['D']    #distortion_coefficients   #1x5
    kd=k['kdata'] # kd is 3x3 matrix in yaml file
    kd=np.reshape(kd,(3,3))  # so reshape
    dd=d['ddata'] # dd is 1x5 matrix in yaml file, so not reshape
    matrix_coefficients=np.array(kd)
    distortion_coefficients=np.array(dd)
  f.close()
  # main function
  main(sys.argv)
