#include "ros/ros.h"
#include <sstream>
#include <math.h>
#include <stdio.h>
#include <vector>

#include "../include/starline/config.h"

double cal_line_dist(double x0,double y0,double x1,double y1)
{
    double dist = 0.0;

    dist = sqrt(pow(x0 - x1,2)+pow(y0 - y1,2));
    return dist;
}

double adjust_theta(double th)
{
    if(th >= PI)
    {
        while(th >= PI)
        {
            th -= 2*PI;
        }
    }
    else if(th < -PI)
    {
        while(th < -PI)
        {
            th += 2*PI;
        }
    }
    return th;
}


