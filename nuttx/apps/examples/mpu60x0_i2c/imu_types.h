#ifndef _IMU_TYPES_H
#define _IMU_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

/********************************************飞机状态记录组SENSER_OFFSET_FLAG**********************************************/
extern uint8_t   SENSER_OFFSET_FLAG; //标志位组

//每一位对应功能
#define GYRO_OFFSET 0x01 //第一位陀螺仪校准标志位
#define ACC_OFFSET 0x02  //第二位加速度校准标志位
#define BAR_OFFSET 0x04  //第三位气压计校准标志位
#define MAG_OFFSET 0x08  //第四位磁力计校准标志位
#define FLY_ENABLE 0x10  //第五位解锁上锁
#define WiFi_ONOFF 0x20  //第六位WiFi开关
#define FLY_MODE   0x40  //第七位模式选择(0:无头模式(默认) 1:有头模式)

//对 SENSER_OFFSET_FLAG 的位的操作
#define SENSER_FLAG_SET(FLAG)   SENSER_OFFSET_FLAG|=FLAG                //标志位置1
#define SENSER_FLAG_RESET(FLAG) SENSER_OFFSET_FLAG&=~FLAG               //标志位值0
#define GET_FLAG(FLAG)         (SENSER_OFFSET_FLAG&FLAG)==FLAG ? 1 : 0  //获取标志位状态


/**************************************************************************************************************************/

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define Byte0(data)       ( *( (char *)(&data)		) )
#define Byte1(data)       ( *( (char *)(&data) + 1) )
#define Byte2(data)       ( *( (char *)(&data) + 2) )
#define Byte3(data)       ( *( (char *)(&data) + 3) )

//三轴整型（MPU9250原始数据）
typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}INT16_XYZ;

//三轴浮点型
typedef struct
{
	float X;
	float Y;
	float Z;
}FLOAT_XYZ;

//姿态解算后的角度
typedef struct
{
	float rol;
	float pit;
	float yaw;
}FLOAT_ANGLE;

typedef struct
{
	double rol;
	double pit;
	double yaw;
}DOUBLE_ANGLE;

typedef struct 
{
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
  int16_t temp;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
}sensor_data_s_test;


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif