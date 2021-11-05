/****************************************************************************
 * apps/examples/spl06_i2c/spl06_i2c_main.c
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

#include <nuttx/sensors/spl06.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPU_DEV_PATH "/dev/spl0"

/****************************************************************************
 * Private Types
 ****************************************************************************/

float spl_buf[3];             // 用于存储读回的数据

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
  int ret; 
  float pressure;        //处理后的压力值   单位hPa
  float temperature;     //处理后的温度     单位摄氏度
  float ASL;             //处理后的海拔高度  单位米

  filp_sensor = open(MPU_DEV_PATH, O_RDWR);
  if(filp_sensor == NULL)
  {
      printf("Unable to create file\n");
      return -ENONET;
  }
  _alert("spl06_i2c_main:main   open success \r\n");// add by herc   2021.11.4
  
  for(; ; )
  {
      //_alert("spl06_i2c_main:main   ready  read \r\n");// add by herc   2021.11.4
      ret = read(filp_sensor, spl_buf, sizeof(spl_buf));
      if(ret < 0) 
        printf("READ failed %d \n",ret);
      //_alert("spl06_i2c_main:main   read  success \r\n");// add by herc   2021.11.4
      pressure    = spl_buf[0];       //单位hPa
      temperature = spl_buf[1];       //单位摄氏度
      ASL         = spl_buf[2];       //单位米
     
      printf("pressure :%f     temperature:%f     ASL:%f\n",
             (float)pressure,
             (float)temperature,
             (float)ASL);

      sleep(3);
  }
  
  fclose(filp_sensor);
  return EXIT_SUCCESS;
}
