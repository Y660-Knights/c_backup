/*
 ============================================================================
 Name        : main.c
 Author      : knights
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>

#include "imu.h"

int main(void) {
	struct position * pos;
	uint16_t i;

	for(i = 0; i < 64*30; i ++)
	//for(;;)
	{
		prepare_data(0,0,0,-1,-2,-3);
		pos = IMU_update();
		if(pos != 0)
			printf("yaw:%0.2f,roll:%0.2f,pitch:%0.2f\r\n",pos->yaw,pos->roll,pos->pitch); /* prints !!!Hello World!!! */
		//printf("!!!Hello World!!! "); /* prints !!!Hello World!!! */
	}

	return EXIT_SUCCESS;
}
