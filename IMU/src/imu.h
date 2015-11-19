/*
 * imu.h
 *
 *  Created on: 2015年11月18日
 *      Author: knights
 */

#ifndef IMU_H_
#define IMU_H_

#include "stdint.h"

struct position
{
	float yaw;
	float roll;
	float pitch;
};

struct position* IMU_update();
void prepare_data(int16_t gx, int16_t gy, int16_t gz, int16_t ax, int16_t ay, int16_t az);

#endif /* IMU_H_ */
