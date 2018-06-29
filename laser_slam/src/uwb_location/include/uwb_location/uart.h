#ifndef UWB_SERIAL_H_
#define UWB_SERIAL_H_

#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include <termios.h>
#include "errno.h"
#include "string.h"
#include <pthread.h>
#include "sys/time.h"
#include "sys/epoll.h"

#define MAX_LEN (10)

class UwbSerial
{
	public:
		UwbSerial();
		virtual ~UwbSerial();

		void set_speed(int fd, int speed);
		int set_parity(int fd,int databits,int stopbits,int parity);
		int open_com_device(char *dev);

};

#endif