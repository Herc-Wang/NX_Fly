/****************************************************************************
 * apps/examples/mpu60x0_i2c/mpu60x0_i2c_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>

#include <nuttx/sensors/mpu60x0.h>
#include "imu_types.h"
#include "imu.h"
#include "imu_filter.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define M_PI        3.14159265358979323846 

#define MPU_DEV_PATH "/dev/imu0"

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

INT16_XYZ	 GYRO_OFFSET_RAW={0,0,0},ACC_OFFSET_RAW={0,0,0};	  //零漂数据
uint8_t    SENSER_OFFSET_FLAG;                       //传感器校准标志位
/****************************************************************************
 * Private Data
 ****************************************************************************/

sensor_data_s_test  MPU_Data_Conv;

DOUBLE_ANGLE Att_Angle_Data;                  //飞机姿态数据
FLOAT_XYZ 	Gyr_rad,Gyr_radold;	              //把陀螺仪的各通道读出的数据，转换成弧度制
FLOAT_XYZ 	MPU_ACC_Filt, MPU_GRY_Filt, Acc_filtold;	  //滤波后的各通道数据

static uint8_t MPU_buff[14];                  //加速度、温度、陀螺仪原始数据

INT16_XYZ	 MPU_ACC_RAW,MPU_GYRO_RAW;	//读取值原始数据   

char data_to_send[50];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void ANO_Send_IMU(double angle_rol, double angle_pit, double angle_yaw, int32_t alt, uint8_t FLY_ENABLEl, uint8_t armed)
{
    int _cnt=0;
    int sum = 0;
    int16_t _temp;
    double _temp_d;

    data_to_send[_cnt++]=0xAA;    
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x01;      //func   
    data_to_send[_cnt++]=0x0C;      //len

    _temp_d = angle_rol * 1000 * 6; 
    _temp   = (int16_t)_temp_d;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    //printf("angle_rol = %f,  _temp_d*1000 = %f,  _temp = %d\r\n", angle_rol, _temp_d, _temp);
    //printf("_temp = %d,  data_to_send[4] = %x ,  data_to_send[5] = %x\r\n", _temp, data_to_send[_cnt-2], data_to_send[_cnt-1]);
    _temp_d = angle_pit * 1000 * 6; 
    _temp   = (int16_t)_temp_d;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    //printf("angle_pit = %f,  _temp_d*1000 = %f,  _temp = %d\r\n", angle_pit, _temp_d, _temp);
    //printf("_temp = %d,  data_to_send[6] = %x ,  data_to_send[7] = %x\r\n", _temp, data_to_send[_cnt-2], data_to_send[_cnt-1]);

    _temp_d = angle_yaw * 1000 * 6; 
    _temp   = (int16_t)_temp_d; 
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    //printf("angle_yaw = %f,  _temp_d*1000 = %f,  _temp = %d\r\n", angle_yaw, _temp_d, _temp);
    //printf("_temp = %d,  data_to_send[8] = %x ,  data_to_send[9] = %x\r\n", _temp, data_to_send[_cnt-2], data_to_send[_cnt-1]);
    //alt_use
    data_to_send[_cnt++]=BYTE0(alt);
    data_to_send[_cnt++]=BYTE1(alt);
    data_to_send[_cnt++]=BYTE2(alt);
    data_to_send[_cnt++]=BYTE3(alt);
    //fly_model
    data_to_send[_cnt++]=FLY_ENABLEl;
    //armed
    data_to_send[_cnt++]=armed;
    //puts_MY(data_to_send);

    for(int i=0;i<_cnt;i++)
      sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    sum = 0;

    ANO_Send_Data(data_to_send, _cnt);
}


void ANO_Send_Data(u_char *dataToSend , int length)
{ 
    FILE *stream_std = stdout;
    
    lib_take_semaphore(stream_std);
    _NX_WRITE(stream_std->fs_fd, data_to_send, length);
    lib_give_semaphore(stdout);
}



/****************************************************************************
 * Name: main
 ****************************************************************************/

int main(int argc, char *argv[])
{
  FILE *filp_sensor;

//  struct sensor_data_s_test *sensor_data;
  int ret; 
  int read_time_tmp = 0;

  
  
//  sensor_data = (struct sensor_data_s_test *)malloc(sizeof(struct sensor_data_s_test));
//  memset(sensor_data, 0, sizeof(* sensor_data));

  filp_sensor = open(MPU_DEV_PATH, O_RDWR);
  if(filp_sensor == NULL)
  {
      printf("Unable to create file\n");
      return -ENODEV;
  }

  SENSER_FLAG_SET(GYRO_OFFSET);//校准陀螺仪 加速度计
  SENSER_FLAG_SET(ACC_OFFSET);//校准陀螺仪 加速度计

  for(; ; )
  {
    
      ret = read(filp_sensor, MPU_buff, sizeof(MPU_buff));
      if(ret < 0) 
        printf("READ failed %d \n",ret);

//      if(++read_time_tmp > MPU_AVG_TIME)
//      {
//          read_time_tmp = 0;
//
//        
//
//          IMUupdate(&Gyr_rad,&Acc_filt,&Att_Angle_Data); //四元数姿态解算
//
//          ANO_Send_IMU(Att_Angle_Data.rol, Att_Angle_Data.pit, Att_Angle_Data.yaw, 0x1234, 1, 2);//通过串口发送欧拉角数据到匿名上位机V4.34
//
//          accData[0] = 0;accData[1] = 0;accData[2] = 0;tempData = 0;gyroData[0] = 0;gyroData[1] = 0;gyroData[2] = 0;
//          
//      }

      MPU_RAWDataProcess(&MPU_ACC_RAW, &MPU_GYRO_RAW, MPU_buff);      //对MPU6050进行处理，减去零偏。如果没有计算零偏就计算零偏
      //if( 0 == GET_FLAG(GYRO_OFFSET) && 0 == GET_FLAG(ACC_OFFSET))
      {
          if(0 == SortAver_FilterXYZ(&MPU_ACC_RAW, &MPU_ACC_Filt, 30))        //对加速度原始数据进行去极值滑动窗口滤波
          {
              //加速度AD值 转换成 米/平方秒
              MPU_ACC_Filt.X = (float)MPU_ACC_Filt.X * ACC_GAIN * G;
              MPU_ACC_Filt.Y = (float)MPU_ACC_Filt.Y * ACC_GAIN * G;
              MPU_ACC_Filt.Z = (float)MPU_ACC_Filt.Z * ACC_GAIN * G;
              //printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",MPU_ACC_Filt.X,MPU_ACC_Filt.Y,MPU_ACC_Filt.Z);

              //陀螺仪AD值 转换成 弧度/秒
              MPU_GRY_Filt.X = (float)MPU_GYRO_RAW.X * GYRO_GR;
              MPU_GRY_Filt.Y = (float)MPU_GYRO_RAW.Y * GYRO_GR;
              MPU_GRY_Filt.Z = (float)MPU_GYRO_RAW.Z * GYRO_GR;

              IMU_update(&MPU_GRY_Filt, &MPU_ACC_Filt, &Att_Angle_Data); //四元数姿态解算

              ANO_Send_IMU(Att_Angle_Data.rol, Att_Angle_Data.pit, Att_Angle_Data.yaw, 0x1234, 1, 2);//通过串口发送欧拉角数据到匿名上位机V4.34
          }
      }


      //usleep(10);
      //sleep(1);
  }
  
  fclose(filp_sensor);
  return EXIT_SUCCESS;
}
