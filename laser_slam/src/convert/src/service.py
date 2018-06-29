#!/usr/bin/env python
#from ._ConvertImage import *
import roslib
roslib.load_manifest('convert')
from convert.srv import *
import Image
import rospy


def handle_convert_pgm_2_png(req):
	#print ('%s',req.scene_name)
    im = Image.open(req.scene_name+'/map/'+req.map_name+'.pgm')
    im.save(req.scene_name+'/map/'+req.map_name+'.png')
    #print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return 0
def convert_pgm_2_png_server():
    rospy.init_node('convert_image')
    print ('Ready to convert the image.')
    s = rospy.Service('convert_pgm_2_png', PGM2PNG, handle_convert_pgm_2_png)
    rospy.spin()

if __name__ == "__main__":
    convert_pgm_2_png_server()
