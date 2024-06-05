#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define WR_VALUE _IOW('a', 'a', int32_t*)
#define RD_VALUE _IOR('a', 'b', int32_t*)
int glob_var;
int main()
{
	int fd;
	int32_t value[3], number;
	double val[3];
	printf("Opening Char Driver - MPU6050\n");
	fd = open("/dev/MPU6050", O_RDWR);
	if(fd < 0) {
		printf("Cannot open device file %d\n",fd);
		return 0;
	}

	printf("Reading accelerometer data from Driver\n");
	
	/*
	 * fd - file descriptor 
	 * RD_VALUE - 
	 * value - Actual value obtained from the kernel space */

	ioctl(fd, RD_VALUE, (int32_t*) &value);
	for(int i = 0; i < 3; i++){
		val[i] = value[i];
		val[i] /= 16384.0;
	}

	printf("X: %lf Y: %lf Z: %lf \n", val[0],val[1],val[2]);
	return 0;
}
