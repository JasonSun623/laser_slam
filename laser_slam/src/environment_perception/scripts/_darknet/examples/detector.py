# Stupid python path shit.
# Instead just add darknet.py to somewhere in your python path
# OK actually that might not be a great idea, idk, work in progress
# Use at your own risk. or don't, i don't care

import sys, os
sys.path.append('/home/cc/Desktop/Object_Detection/using_darknet/darknet-master/python')

import darknet as dn
import pdb

dn.set_gpu(0)
net = dn.load_net("/home/cc/Desktop/Object_Detection/using_darknet/darknet-master/cfg/yolov3.cfg", "/home/cc/Desktop/Object_Detection/using_darknet/darknet-master/yolov3.weights", 0)
meta = dn.load_meta("/home/cc/Desktop/Object_Detection/using_darknet/darknet-master/cfg/coco.data")
r = dn.detect(net, meta, "/home/cc/Desktop/Object_Detection/using_darknet/darknet-master/data/kite.jpg")
print r


