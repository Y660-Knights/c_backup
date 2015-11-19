/*
 * imu.c
 *
 *  Created on: 2015年11月18日
 *      Author: knights
 */

#include "imu.h"
#include "math.h"

#define MaxFilterCount		64
#define Gyro_G				9.8f
#define	Sample_period		0.1f

//输出角度
struct position p;

static uint8_t	filter_cnt = 0;
static int16_t	ax_buf[MaxFilterCount] = {0};
static int16_t	ay_buf[MaxFilterCount] = {0};
static int16_t	az_buf[MaxFilterCount] = {0};

//加速度平均
static float ax_avg = 0;
static float ay_avg = 0;
static float az_avg = 0;
//角加速度
static double gyro_x_i = 0;
static double gyro_y_i = 0;
static double gyro_z_i = 0;

//四元素
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float exInt = 0, eyInt = 0, ezInt = 0;
//为什么要用陀螺仪的积分呢？ 加速度积分就是速度
#define Kp 2.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f                // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.1f                // half the sample period

struct position* IMU_update()
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float gx, gy, gz, ax, ay, az;

	gx = gyro_x_i;
	gy = gyro_y_i;
	gz = gyro_z_i;
	ax = ax_avg;
	ay = ay_avg;
	az = az_avg;

	if(ax ==0 || ay == 0 || az == 0)
	{
		return 0;
	}

	// normalise the measurements 把加计的三维向量转成单位向量。
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
/*
这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。 */
	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
/*
axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积
分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对
陀螺的纠正量会直接体现在对机体坐标系的纠正。
*/
	// error is sum of cross product between reference direction of field and direction  measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	// adjusted gyroscope measurements 用叉积误差来做PI修正陀螺零偏
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	// integrate quaternion rate and normalise 四元数微分方程
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
	// normalise quaternion 四元数规范化
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	p.yaw += gz*Gyro_G*0.002f;

	p.roll = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
	p.pitch = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll

	return &p;
}

void prepare_data(int16_t gx, int16_t gy, int16_t gz, int16_t ax, int16_t ay, int16_t az)
{
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

	ax_buf[filter_cnt] = ax;
	ay_buf[filter_cnt] = ay;
	az_buf[filter_cnt] = az;


	for(i=0;i<MaxFilterCount;i++)
	{
		temp1 += ax_buf[i];
		temp2 += ay_buf[i];
		temp3 += az_buf[i];
	}

	ax_avg = temp1 / MaxFilterCount;
	ay_avg = temp2 / MaxFilterCount;
	az_avg = temp3 / MaxFilterCount;
	filter_cnt++;
	if(filter_cnt >= MaxFilterCount)
		filter_cnt = 0;

	gyro_x_i = gx * Gyro_G * Sample_period;
	gyro_y_i = gy * Gyro_G * Sample_period;
	gyro_z_i = gz * Gyro_G * Sample_period;

}


