#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np
import sys, os, select, termios, tty

import rospy
from std_msgs.msg import String

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CameraInfo
import json
from collections import OrderedDict
import math
import copy

from ReadCalibraPara import *
import rospkg
rospack=rospkg.RosPack()
_PackPath=rospack.get_path('environment_perception')
sys.path.append(_PackPath+'/scripts/detection/')
from detection.detector import *
sys.path.append(_PackPath+'/scripts/config/')
from setup import yaml_path

#just visual laser frame(ranges and intens)
def visual_Laser(laserFrame,sampleStep):

  laseR=laserFrame.LaserRanges
  laserIntens=laserFrame.intens
  agmin=laserFrame.agMin
  angmax=laserFrame.agMax
  angincre=laserFrame.agIncre
  canvas=np.zeros((1200,2000,3),dtype="uint8")
  cv2.namedWindow("LaserLines",0)
  for i in range(0,len(laseR),sampleStep):
 # for i in range(290,315):
    p1=(1000,1200)
    etha=agmin+angincre*i
    
    L=laseR[i]*500
    i_intens=laserIntens[i]

    color=(255,int(0.4*float(i_intens)),int(0.8*float(i_intens)))
    if L>0:      
       #print "etha: ",etha
       p2_x=-int(L*math.sin(etha))+1000
       p2_y=1200-int(L*math.cos(etha))       
       cv2.line(canvas,p1,(p2_x,p2_y),color,4)
  cv2.imshow("LaserLines",canvas)
  cv2.waitKey(1)

#undistorted points
def undistortPoints(points,CaPa):
  ifx=1./CaPa.mtx[0][0]
  ify=1./CaPa.mtx[1][1]
  cx=CaPa.mtx[0][2]
  cy=CaPa.mtx[1][2]  
  k1=CaPa.dist[0][0]
  k2=CaPa.dist[0][1]
  p1=CaPa.dist[0][2]
  p2=CaPa.dist[0][3]
  undisPoints=[]
  for i in range(len(points)):

      x=float(points[i][0]-cx)*ifx
      y=float(points[i][1]-cy)*ify
      x0=x
      y0=y
      for j in range(5):
          r2=float(x*x)+float(y*y)
          icdist=1./(1+k1*r2+k2*r2*r2)
          delta_x=2.0*p1*x*y+p2*(r2+2.0*x*x)
          delta_y=p1*(r2+2.0*y*y)+2.0*p2*x*y
          x=float(x0-delta_x)*icdist
          y=float(y0-delta_y)*icdist
      undisPoints.append((float(x)/ifx+cx,float(y)/ify+cy))
  return  undisPoints
      

class image_converter:
  def __init__(self):
      self.bridge = CvBridge()
      self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
      self.came_headseq_sub = rospy.Subscriber("/usb_cam/camera_info",CameraInfo,self.callback_info)

  def callback(self,data):
     try:
         self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

     except CvBridgeError as e:
       print e
  
  def callback_info(self,data):
     try:
        self.seq  = data.header.seq
        self.stamp_secs=data.header.stamp.secs
        self.stamp_nsecs=data.header.stamp.nsecs
     except :
       print "No CameHead Info !"  

     
class getLaserFrame:
  def __init__(self):
     
      self.Laser_sub = rospy.Subscriber("/scan",LaserScan,self.callback)
  def callback(self,data):
     try:
         self.LaserRanges=data.ranges
         self.agMax=data.angle_max
         self.agMin=data.angle_min
         self.agIncre=data.angle_increment
         self.intens=data.intensities
         
     except:
       print "laser error"

class fromCametoLaser:
  def __init__(self,laser_maxAngle,laser_minAngle,laser_increAngle):
      self.laser_maxAngle=laser_maxAngle
      self.laser_minAngle=laser_minAngle
      self.laser_increAngle=laser_increAngle
 
  def getCameAngleIndex(self,Radian0,Radian1,LaserNSample):
       cameTolaserOffset=0#-0.08
       Radian0=Radian0-cameTolaserOffset
       Radian1=Radian1-cameTolaserOffset
       if Radian0>self.laser_maxAngle:
          Radian0=self.laser_maxAngle
       elif Radian0< self.laser_minAngle:
          Radian0= self.laser_minAngle
       if Radian1>self.laser_maxAngle:
          Radian1=self.laser_maxAngle
       elif Radian1< self.laser_minAngle:
          Radian1= self.laser_minAngle
       index0=int(float(Radian0-self.laser_minAngle)/self.laser_increAngle)
       index1=int(float(Radian1-self.laser_minAngle)/self.laser_increAngle)
       
       if index0>=index1:
           tempindex=index0
           index0=index1
           index1=tempindex
       
       if index0<0:
          index0=0
       if index1>=LaserNSample:
          index1=LaserNSample-1
       
       self.indexs=[index0,index1]

  def getObjRange(self,LaserRanges):       
       ranges=LaserRanges[self.indexs[0]:(self.indexs[1]+1)]
       NA=np.array(ranges)
       #print "lase ranges ",NA
       NAnotZero=np.where(NA>0.02)
       NAnotZeroA=NA[NAnotZero]
       if len(NAnotZeroA)>0:
          minRange=np.min(NAnotZeroA)
       else:
          minRange=-0.1
         
       return minRange

class Came_Info_struct():
      def __init__(self,seq,secs,nsecs):
         self.seq=seq
         self.stamp_secs=secs
         self.stamp_nsecs=nsecs

def getJsonMsg(ReciveTime,SendTime,came_info,results,H,W,C2L,laseR,CaliYaml):
                  
         classNames=[]
         confs=[]
         boxes=[]
         for i in range(len(results)):
                classNames.append(results[i][0])
                confs.append(results[i][1])
                box=(results[i][2:])
                x1 = int(round((box[0] - box[2]/2.0)))
                if x1<1:
                   x1=1
                y1 = int(round((box[1] - box[3]/2.0)))
                if y1<1:
                   y1=1
                box[0]=x1
                box[1]=y1
                boxes.append(box)
                

         obj_list=[]
         
         OBJ_index=[]
         for k in range(len(classNames)):
             if classNames[k]=="person":
                 OBJ_index.append(k)
         OBJ_NUM=len(OBJ_index)
         LaserNSample=len(laseR)
         for i in range(OBJ_NUM):             
            j=OBJ_index[i]
            oid=i
            ocl=0
            oconf=round(confs[j],3)
            ibox=boxes[j]
            points=[(ibox[0],ibox[1]),(ibox[2]+ibox[0],ibox[3]+ibox[1])]
            correctPoints=CaliYaml.undistortPoints(points)

            tan0=float(correctPoints[0][0]-CaliYaml.cx)/CaliYaml.fx
            tan1=float(correctPoints[1][0]-CaliYaml.cx)/CaliYaml.fx
            Radian0=math.atan(-tan0)
            Radian1=math.atan(-tan1)
            C2L.getCameAngleIndex(Radian1,Radian0,LaserNSample)
            objR=C2L.getObjRange(laseR)
            L=OrderedDict()
            L["object_id"]=oid
            L["object_bbox"]=[int(ibox[0]),int(ibox[1]),int(ibox[2]),int(ibox[3])]
            L["object_class"]=ocl
            L["object_confidence"]=oconf
            L["object_Radian"]=[Radian0,Radian1]#[-1,-1]
            L["object_distance"]=objR
            L["Object_speed"]=(-1,-1)
            obj_list.append(L)

         datas=OrderedDict()
         datas["object_num"]=OBJ_NUM
         datas["camera_seq"]=came_info.seq
         datas["camera_stamp_secs"]=came_info.stamp_secs
         datas["camera_stamp_nsecs"]=came_info.stamp_nsecs
         datas["receive_time"]=str(ReciveTime)
         datas["send_time"]=str(SendTime)
         datas["object_list"]=obj_list
                  
         content=OrderedDict()
         content["sub_name"]="environment_perception"
         content["data"]=datas
         JsonMsg=json.dumps(content)
         #rospy.loginfo(JsonMsg)
         return JsonMsg
      
if __name__ == '__main__':
    
    #Detector=cv_Object_Detection()
    #cap = cv2.VideoCapture(0)
    #while True:
    #      res, img = cap.read()
    #      if res:
    #          r=Detector.do_detection_Img(img)


    #sub came image
    ic = image_converter()

    #sub laser frame
    getLF =getLaserFrame()
    
    #init ros node       
    rospy.init_node('environment_perception')
    pub = rospy.Publisher('environment_perception_node', String, queue_size=15)
    
    #version
    rospy.set_param('environment_perception_version','V1.0.1')
    
    #detector
    OBJ=cv_Object_Detection()
    
    #read cam_caliba_para from yaml
    CaliYaml=ReadCalibraYaml(yaml_path)   
    #set detection frequency   
    rate = rospy.Rate(5)
    #get some init para
    CamLaser=False
    # main loop
    while not rospy.is_shutdown():
            try:
               frame=ic.cv_image
               came_info=Came_Info_struct(ic.seq,ic.stamp_secs,ic.stamp_nsecs)
               
            except :
               rospy.loginfo("Camera Is Not Ready!!")
               continue
       
            if not CamLaser:
               H=frame.shape[0]
               W=frame.shape[1]
               try:
                  C2L=fromCametoLaser(getLF.agMax,getLF.agMin,getLF.agIncre)
                  CamLaser=True
               except :
                  rospy.loginfo("Laser Is Not Ready!!")
               continue
            
            #just visual laser ranges
            #visual_Laser(getLF,2)
            #get imge receive time(ROS time)
            ReciveTime=rospy.Time.now()
     
            #get laser ranges
            laseR=getLF.LaserRanges
            #detect the image frame
            results=OBJ.do_detection_Img(frame)
            #get massage sended time(ROS time)
            SendTime=rospy.Time.now()

            #pub json message
            pubMsg=getJsonMsg(ReciveTime,SendTime,came_info,results,H,W,C2L,laseR,CaliYaml)
            pub.publish(pubMsg)
            rate.sleep()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")





    
   













