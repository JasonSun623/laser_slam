# Stupid python path shit.
# Instead just add darknet.py to somewhere in your python path
# OK actually that might not be a great idea, idk, work in progress
# Use at your own risk. or don't, i don't care
import cv2
from ctypes import *
import math
import random
import sys, os
sys.path.append('/home/cc/Desktop/Object_Detection/using_darknet/darknet-master/python/')

import darknet as dn

import time
from ctypes import *
import numpy as np
import copy



def detect_np(net, meta, np_img, thresh=.5, hier_thresh=.5, nms=.45):
    im = nparray_to_image(np_img)
    boxes = make_boxes(net)
    probs = make_probs(net)
    num = num_boxes(net)
    network_detect(net, im, thresh, hier_thresh, nms, boxes, probs)
    res = []
    for j in range(num):
        for i in range(meta.classes):
            if probs[j][i] > 0:
                res.append((meta.names[i], probs[j][i], (boxes[j].x, boxes[j].y, boxes[j].w, boxes[j].h)))
    res = sorted(res, key=lambda x: -x[1])
    free_image(im)
    free_ptrs(cast(probs, POINTER(c_void_p)), num)
    return res


def nparray_to_image(img):
    data = img.ctypes.data_as(POINTER(c_ubyte))
    image = dn.ndarray_image(data, img.ctypes.shape, img.ctypes.strides)

    return image



def c_array(ctype, values):
    t1=time.time()
    if not values.flags['C_CONTIGUOUS']:
     values = np.ascontiguous(values, dtype=values.dtype)
    arr = (ctype*len(values))()
    
    
    
    arr[:] =values
    """
    c_p=POINTER(ctype)
    data=np.array(values)
    data=data.astype(np.float32)
    arr=data.ctypes.data_as(c_p)
    """
    
    t2=time.time()
    print "c_array ",(t2-t1)*1000," ms"
    return arr




def array_to_image(arr):

    arr = arr.transpose(2,0,1)
    
    c = arr.shape[0]
    h = arr.shape[1]    
    w = arr.shape[2]
   
    arr = (arr/255.0).flatten()
   
    data =c_array(c_float, arr)

    im = dn.IMAGE(w,h,c,data)
    return im

def plot_boxes(img, data):
    width = img.shape[1]
    height = img.shape[0]
    for i in range(len(data)):
        className=data[i][0]
        conf=data[i][1]
        box = data[i][2:]
        x1 = int(round((box[0] - box[2]/2.0)))
        if x1<1:
           x1=1
        y1 = int(round((box[1] - box[3]/2.0)))
        if y1<1:
           y1=1
        x2 = int(round((box[0] + box[2]/2.0)))
        if x2>(width-1):
           x2=width-2
        y2 = int(round((box[1] + box[3]/2.0)))
        if y2>(height-1):
            y2=height-2

        rgb = (0, 255, 0)
        
        cv2.putText(img,className+": "+str(conf), (x1,y1), cv2.FONT_HERSHEY_SIMPLEX, 1, rgb, 1)
        cv2.rectangle(img, (x1,y1), (x2,y2), rgb, 1)
    cv2.imshow("Result",img)
    cv2.waitKey(1)


   
def detect2(net, meta, cvim, thresh=.7, hier_thresh=.5, nms=.45):
    t1=time.time()
    im = nparray_to_image(cvim)
    t2=time.time()
    print "cv2 to IMAGE: ",((t2-t1)*1000)
    num = c_int(0)
    pnum = pointer(num)
    dn.predict_image(net, im)
    dets = dn.get_network_boxes(net, im.w, im.h, thresh, hier_thresh, None, 0, pnum)
    num = pnum[0]
    if (nms): dn.do_nms_obj(dets, num, meta.classes, nms);

    res = []
    for j in range(num):
        for i in range(meta.classes):
            if dets[j].prob[i] > 0:
                b = dets[j].bbox
                res.append([meta.names[i], dets[j].prob[i], b.x, b.y, b.w, b.h])
    res = sorted(res, key=lambda x: -x[1])
    dn.free_image(im)
    dn.free_detections(dets, num)
    return res


if __name__ == "__main__":

# Darknet
 net = dn.load_net("/home/cc/Desktop/Object_Detection/using_darknet/darknet-master/Test/Net510.cfg", "/home/cc/Desktop/Object_Detection/using_darknet/darknet-master/Test/Weights/Net510_final.weights", 0)
 meta = dn.load_meta("/home/cc/Desktop/Object_Detection/using_darknet/darknet-master/Test/Test.data")

 
 cap=cv2.VideoCapture(0)
 #arr = cv2.imread("/home/cc/Desktop/Object_Detection/using_darknet/darknet-master/data/dog.jpg")
# OpenCV
 while True:
  
  #arr= cv2.imread("/home/cc/Desktop/Object_Detection/using_darknet/darknet-master/Test/TestImgs/GZ4.jpg")
  ret,arr=cap.read()
  im0=cv2.resize(arr,(288,288))
  #im = array_to_image(im0)


  #dn.rgbgr_image(im)

  tt3=time.time()
  r = detect2(net, meta, im0)
  tt4=time.time()
  print "detection all cost: ",int((tt4-tt3)*1000)
  print r
  plot_boxes(im0, r)

  
