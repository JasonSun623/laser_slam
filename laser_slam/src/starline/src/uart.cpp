/*
 gavin 2016-2-25
 */
#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <sstream>
#include <math.h>
#include <stdio.h>     
#include <stdlib.h>     
#include <unistd.h>     
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>      
#include <termios.h>   
#include <errno.h>     
#include <string.h>


/*
*@brief  
*@param  fd    
*@param  speed 
*@return  void
*/
static int speed_arr[] = { B38400, B19200, B115200, B9600, B4800, B2400, B1200, B300 };
static int name_arr[] = {38400,  19200,  115200,  9600,  4800,  2400,  1200,  300 };

void set_speed(int fd, int speed)
{
    unsigned int i; 
    int status; 
    struct termios Opt;
    
    tcgetattr(fd, &Opt); 
    for(i = 0;i < sizeof(speed_arr)/sizeof(int);i++) 
    { 
        if(speed == name_arr[i]) 
        {     
            tcflush(fd,TCIOFLUSH);     
            cfsetispeed(&Opt,speed_arr[i]);  
            cfsetospeed(&Opt,speed_arr[i]);   
            status = tcsetattr(fd,TCSANOW,&Opt);  
            if(status != 0) 
            {        
                ROS_ERROR("tcsetattr fd");  
                return;     
            }    
            tcflush(fd,TCIOFLUSH);   
        }  
    }
}

/**
*@brief   
*@param  fd  
*@param  databits 
*@param  stopbits 
*@param  parity 
*/
int set_parity(int fd,int databits,int stopbits,int parity)
{ 
    struct termios options; 
    if(tcgetattr( fd,&options)  !=  0) 
    { 
        ROS_ERROR("SetupSerial 1");     
        return 0;  
    }
    options.c_cflag &= ~CSIZE; 

    switch (databits) 
    {   
        case 7:        
            options.c_cflag |= CS7; 
            break;
        case 8:     
            options.c_cflag |= CS8;
            break;   
        default:    
        ROS_ERROR("Unsupported data size\n"); 
        return -1;  
    }

    switch (parity) 
    {   
        case 'n':
        case 'N':    
            options.c_cflag &= ~PARENB;   /* Clear parity enable */
            options.c_iflag &= ~INPCK;     /* Enable parity checking */ 
            break;  
        case 'o':   
        case 'O':     
            options.c_cflag |= (PARODD | PARENB); /* */  
            options.c_iflag |= INPCK;             /* Disnable parity checking */ 
            break;  
        case 'e':  
        case 'E':   
            options.c_cflag |= PARENB;     /* Enable parity */    
            options.c_cflag &= ~PARODD;   /* */     
            options.c_iflag |= INPCK;       /* Disnable parity checking */
            break;
        case 'S': 
        case 's':  /*as no parity*/   
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;  
        default:   
            ROS_ERROR("Unsupported parity\n");    
            return -1;  
    }  
    
    switch (stopbits)
    {   
        case 1:    
            options.c_cflag &= ~CSTOPB;  
            break;  
        case 2:    
            options.c_cflag |= CSTOPB;  
               break;
        default:    
             ROS_ERROR("Unsupported stop bits\n");  
             return -1; 
    }

    /* Set input parity option */ 
    if (parity != 'n')
    {
        options.c_iflag |= INPCK;
    }
    tcflush(fd,TCIFLUSH);
    options.c_cc[VTIME] = 0; /* 15 seconds*/   
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    options.c_iflag &= ~(ICRNL | IGNCR | IXON | BRKINT | INPCK | ISTRIP);  //0x0d->0x0a
    options.c_lflag  &= ~(ICANON | ISIG | ECHO | ECHOE);  /*Input*/
    options.c_oflag  &= ~OPOST;   /*Output*/
    if (tcsetattr(fd,TCSANOW,&options) != 0)   
    { 
        ROS_ERROR("SetupSerial 3");   
        return -1;  
    } 
    return 0;  
}

/**/
int open_com_device(char *dev)
{
    if(NULL == dev)
    {
        ROS_ERROR("dev NULL!");
        return -1;
    }
    
    int    fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);         //| O_NOCTTY | O_NDELAY    
    if (-1 == fd)    
    {             
        ROS_ERROR("Can't Open Serial Port");        
    }    
    
    return fd;
}

