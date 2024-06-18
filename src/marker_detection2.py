#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cvbridge_tutorials')  # 'cvbridge_tutorials' is package name
import sys
import rospy
import cv2
from std_msgs.msg import String,Int32
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
    ##self.center_pub= rospy.Publisher("center_info",Int32MultiArray,queue_size=1)
    self.error_pub= rospy.Publisher("error_info",Int32,queue_size=1)
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback,queue_size = 1)  # '/usb_cam/image_raw' is usb_cam's topic name
    ##self.twist_pub=rospy.Publisher("twist",Twist,queue_size=1)
 
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
      cv_image1 = cv2.resize(cv_image,dsize=(10,10),interpolation=cv2.INTER_AREA)   #dsize: output image size
      # cv_image=cv2.medianBlur(cv_image,31) # odd,up 1
      #print('cv_image : ',cv_image)
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Change grayscale
      height,width,dimension=cv_image.shape
      #print('height: ',h)
      #print('width: ',w)
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
      ##c = Int32MultiArray()
      c = Int32()
      #a = Twist()
      #if np.all(ids is None):  # If there are markers not found by detector
        #print('Marker undetected')
      
      #verify at least one ArUco marker was detected
      if len(corners) > 0:
        #flatten the ArUco IDs list
        ids=ids.flatten()     #flatten the ArUco ids list and then loop over each of the corners and ids list together
        
        #loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
        ##for i in range(0, len(ids)):
          #extract the marker corners (which are always returned in top-left,top-right,bottom-right,bottom-left order)
          ##corners=corners.reshape((4,2))
          corners=markerCorner.reshape((4,2))    #Each markerCorner is represented by a list of four (x,y)-coordinates
          (topLeft, topRight, bottomRight, bottomLeft) = corners
          
          #convert each of the (x,y)-coordinates pairs to integers
          #because topLeft, topRight, bottomRight, bottomLeft variables are Numpy arrays; however we need to cast them to integer values (int) such that we can use  OpenCV's drawing functions to visualize the markers on our image
          topRight=(int(topRight[0]),int(topRight[1]))
          bottomRight=(int(bottomRight[0]),int(bottomRight[1]))
          bottomLeft=(int(bottomLeft[0]),int(bottomLeft[1]))
          topRight=(int(topLeft[0]),int(topLeft[1]))
          
          #compute and draw the center (x,y) - coordinates of the ArUco marker
          Cx=int((topLeft[0] + bottomRight[0])/2.0)
          Cy=int((topLeft[1] + bottomRight[1])/2.0)
          cv2.circle(cv_image,(Cx,Cy),4,(0,0,255),-1)
          
          rvec, tvec,markerPoints= aruco.estimatePoseSingleMarkers(markerCorner, 0.1, matrix_coefficients, distortion_coefficients)   # markerLength width 0.1m    #rvec: rotation vector  #tvec: translation vector
          (rvec - tvec).any()     # get rid of that nasty numpy value array error
          # inversePerspective
          R, _ = cv2.Rodrigues(rvec) # converts rotation vector to rotation matrix using Rodrigues transformation   #Since rvec is Rodrigues' expression for rotation transformation, Opencv's Rodrigues() function should be used to obtain the actual rotation transformation matrix R.
          #R -> 3x3 matrix
          r1=R[0]  #r1 -> 1x3 vector  #first row of Rotation transformation matrix R
          r2=R[1]  #r2 -> 1x3 vector  #second row of Rotation transformation matrix R
          r3=R[2]  #r3 -> 1x3 vector  #third row of Rotation transformation matrix R
          #R = np.matrix(R).T  # transposition   #get inverse matrix of above rotation matrix
          tvec1=np.reshape(tvec,(3,1))       #tvec1 -> 3x1 matrix
          t1=tvec1[0]            
          t2=tvec1[1]
          t3=tvec1[2]
          a1=np.append(r1,t1)  # add list
          a2=np.append(r2,t2)
          a3=np.append(r3,t3)
          a4=np.array([0,0,0,1])
          t=np.r_[[a1],[a2],[a3],[a4]]  # make 4x4 Homogeneous transformation matrix t(=rotation matrix + translation vector)
          tt=np.linalg.inv(t)  # get inverse Homogeneous transformation matrix
          ct=np.array([tt[0][3],tt[1][3],tt[2][3]])  # camera pose
          ##p.position.x = ct[0]
          print('ct[0]: ',ct[0])    #ct[0] : center_dist1
          ##p.position.y = ct[1]
          ##p.position.z = ct[2]
          
          
          print('markerCorner: ',markerCorner)
          print('Cx: ',Cx)
          print('Cy: ',Cy)
          #cv2.putText(cv_image,str(markerID),(topLeft[0],topLeft[1]-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
          print('markerID: ',markerID)
          print('-------------------------')
          ##aruco.drawDetectedMarkers(cv_image.copy(),corners,ids)  #Draw a square around the markers
          
          ##cv2.drawFrameAxes(cv_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)
         
          #cv2.imshow("Image window", cv_image)
          
          #center_info=[Cx,Cy]
          #c.data=center_info
          
          aruco_center=Cx
          error=aruco_center-int(width/2)
          c.data=error

    except CvBridgeError as e:
      print(e)
    print('img.shape: ',cv_image.shape)
    str='o'  # display camera's center
    cv2.putText(cv_image,str,(320,240),cv2.FONT_HERSHEY_PLAIN,1,(138,43,226),3)  # image, text, position, font, size, color, thickness
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)  # imshow & waitKey is essential. if waitKey doesn't exist, imshow turns off immediately

    try:
      self.pose_pub.publish(p) # After working with opencv image, convert to ros image again and publish.
      #self.center_pub.publish(c)
      self.error_pub.publish(c)
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
    #rospy.loginfo(center_data)
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
  with open('/home/unicon4/catkin_ws/src/ArUco_marker_detection/aruco/src/image_web/ost1.yaml') as f: # ost.yaml : calibration again
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
