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


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPU_DEV_PATH "/dev/imu0"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sensor_data_s_test
{
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
  int16_t temp;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
};

static uint8_t MPU_buff[14];    //加速度、温度、陀螺仪原始数据

static int16_t accData[3];        //处理后的加速度值
static int16_t tempData;       //处理后的温度值
static int16_t gyroData[3];       //处理后的陀螺仪值
/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 ****************************************************************************/

int main(int argc, char *argv[])
{
  FILE *filp_sensor;
//  struct sensor_data_s_test *sensor_data;
  int ret; 

//  sensor_data = (struct sensor_data_s_test *)malloc(sizeof(struct sensor_data_s_test));
//  memset(sensor_data, 0, sizeof(* sensor_data));

  filp_sensor = open(MPU_DEV_PATH, O_RDWR);
  if(filp_sensor == NULL)
  {
      printf("Unable to create file\n");
      return -ENONET;
  }

  for(; ; )
  {
    
      ret = read(filp_sensor, MPU_buff, sizeof(MPU_buff));
      if(ret < 0) 
        printf("READ failed %d \n",ret);
      //printf("---sizeof(* sensor_data) = %d,     ret = %d\r\n\n",sizeof(MPU_buff),ret);

      {
          accData[0]   = (int16_t)((MPU_buff[0] << 8) | MPU_buff[1]);
          accData[1]   = (int16_t)((MPU_buff[2] << 8) | MPU_buff[3]);
          accData[2]   = (int16_t)((MPU_buff[4] << 8) | MPU_buff[5]);

          tempData     = (int16_t)((MPU_buff[6] << 8) | MPU_buff[7]);

          gyroData[0]  = (int16_t)((MPU_buff[8] << 8) | MPU_buff[9]);
          gyroData[1]  = (int16_t)((MPU_buff[10] << 8) | MPU_buff[11]);
          gyroData[2]  = (int16_t)((MPU_buff[12] << 8) | MPU_buff[13]);
      }
      
      printf("ACCEL x:%d y:%d z:%d   TEMP:%d    GYRO  x:%d y:%d z:%d\n",
             accData[0],
             accData[1],
             accData[2],
             tempData,
             gyroData[0],
             gyroData[1],
             gyroData[2]);

      sleep(1);
  }
  
  fclose(filp_sensor);
  return EXIT_SUCCESS;
}
