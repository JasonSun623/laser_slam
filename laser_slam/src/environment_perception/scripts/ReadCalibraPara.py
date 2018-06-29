import json
import numpy as np
import os
import cv2
import time
import yaml
class ReadCalibraPara():
    def __init__(self, calibFile):
        if not os.path.isfile(calibFile):
            print "not find Calibration file "
        else:
            CalPara = {}
            with open(calibFile, 'r') as json_file:
                CalPara = json.load(json_file)
            self.mtx = np.array(CalPara['mtx'])
            self.dist = np.array(CalPara['dist'])
            self.newcameramtx = np.array(CalPara['newcameramtx'])
            self.mapx,self.mapy=cv2.initUndistortRectifyMap(self.mtx,self.dist,None,self.mtx,(640,480),5)
    def getUndistort(self,origImg,i=0):
        if i==0:          
           disImg=cv2.remap(origImg,self.mapx,self.mapy,cv2.INTER_LINEAR)
        else:
          disImg=cv2.undistort(origImg, self.mtx, self.dist, None, self.newcameramtx)
        return disImg

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

class ReadCalibraYaml():
    def __init__(self, calibFile):
        if not os.path.isfile(calibFile):
            print "not find Calibration file "
        else:
            CalPara = {}
            with open(calibFile, 'r') as yamlFile:
                CalPara = yaml.load(yamlFile)
            self.cam_mtx = np.array(CalPara['camera_matrix']['data'])
            self.dist = np.array(CalPara['distortion_coefficients']['data'])
            self.ifx=1./self.cam_mtx[0]
            self.ify=1./self.cam_mtx[4]
            self.fx=self.cam_mtx[0]
            self.fy=self.cam_mtx[4]
            self.cx=self.cam_mtx[2]
            self.cy=self.cam_mtx[5]  
            self.k1=self.dist[0]
            self.k2=self.dist[1]
            self.p1=self.dist[2]
            self.p2=self.dist[3]
            
    #undistorted points
    def undistortPoints(self,points):
        ifx=self.ifx
        ify=self.ify
        cx=self.cx
        cy=self.cy
        k1=self.k1
        k2=self.k2
        p1=self.p1
        p2=self.p2
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

if __name__ == '__main__':
   f="/home/robot/Downloads/Project/environment_perception/src/environment_perception/scripts/ost.yaml"
   Pa=ReadCalibraYaml(f)
   pp=[(100,200),(400,200)]
   print Pa.undistortPoints(pp)[1][1]
