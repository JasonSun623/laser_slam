import cv2
from ctypes import *
import math
import random
import time
import sys, os
import netbase as dn
import rospkg
rospack=rospkg.RosPack()
_PackPath=rospack.get_path('environment_perception')
sys.path.append(_PackPath+'/scripts/config/')
from setup import net_path,model_path,meta_path
from setup import record_path

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


   
def detect2(net, meta, cvim, thresh=.3, hier_thresh=.5, nms=.4):
    im = nparray_to_image(cvim)
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





class cv_Object_Detection():
    def __init__(self):
        cfgfile=net_path
        weightfile =model_path
        metafile =meta_path
        self.net = dn.load_net(cfgfile, weightfile, 0)
        self.meta = dn.load_meta(metafile)
        self.show=0
        self.writeImg=0

    def do_detection_Img(self,img):
        if self.show:
           SHowImg=img.copy()
        #t1=time.time()
        bboxes = detect2(self.net, self.meta, img)
        #t2=time.time()
        #print "Time cost: ",t2-t1
        if self.writeImg and len(bboxes):
           cv2.imwrite(record_path+str(time.time())+".jpg",img)
           
        if self.show:
           plot_boxes(img,bboxes)


        return bboxes





if __name__ == '__main__':

  Detector=cv_Object_Detection()
  cap = cv2.VideoCapture(0)
  while True:
        res, img = cap.read()
        if res:
            #img=cv2.resize(img,(288,288))
            r=Detector.do_detection_Img(img)

            #print r


