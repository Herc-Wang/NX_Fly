#ifndef _IMU_H
#define _IMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include "imu_types.h"



/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define G			     9.80665f		        // m/s^2	
#define RadtoDeg        57.324841f				//弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad         0.0174533f				//角度到弧度 (角度 * 3.1415/180)

#define Kp_New      0.9f              //互补滤波当前数据的权重
#define Kp_Old      0.1f              //互补滤波历史数据的权重  

#define ACC_GAIN  	0.0004882f				//加速度变成G (初始化加速度满量程-+xg LSBa = 2*x/65535.0)
#define GYRO_GAIN 	0.0609756f				//角速度变成度 (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)
#define GYRO_GR	    0.0010641f			    //角速度变成弧度(3.1415/180 * LSBg)   

//IMU_update() 用到的宏定义             kp=ki=0 就是完全相信陀螺仪
#define Kp 1.50f                         // proportional gain governs rate of convergence to accelerometer/magnetometer
                                         //比例增益控制加速度计，磁力计的收敛速率
#define Ki 0.005f                        // integral gain governs rate of convergence of gyroscope biases  
                                         //积分增益控制陀螺偏差的收敛速度
#define halfT 0.005f                     // half the sample period 采样周期的一半



extern INT16_XYZ GYRO_OFFSET_RAW,ACC_OFFSET_RAW;	  //零漂数据
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
void MPU_RAWDataProcess(INT16_XYZ *MPU_ACC_RAW, INT16_XYZ *MPU_GYRO_RAW, uint8_t MPU_buff[]);
                        
#endif