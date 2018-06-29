import cv2
import numpy as np


def getAngle():
  
  return angle1,angle2

def rangimg_from_leser():


  return distance


def ranging_from_monocular(bbox,cy,fy):
  CameH=0.965
  deltaY=cy-bbox[3]
  if deltaY>0:
     distance=CameH*fy/deltaY
  return distance


def ranging_from_pnp():


  return distance


def get_ObjDistance():


  return distance



