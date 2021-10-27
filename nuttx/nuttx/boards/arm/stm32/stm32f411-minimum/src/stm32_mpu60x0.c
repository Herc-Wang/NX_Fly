/****************************************************************************
 * boards/arm/stm32/stm32f411-minimum/src/stm32_mpu60x0.c
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

#include <stdint.h>
#include <debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include <nuttx/sensors/mpu60x0.h>

//#include "stm32_gpio.h"
//#include "stm32_spi.h"
#include "stm32f411-minimum.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mpu60x0_initialize
 *
 * Description:
 *
 *   Initialize and register's stm32f411-minimum's MPU60x0 IMU. The wiring
 *   isn't configurable, but we use macros anyway because some of the
 *   values are referred to in more than one place. And also, because
 *   that's generally what NuttX does.
 *
 *   In particular, the slave-select pin is defined by us, but
 *   controlled elsewhere as part of the SPI machinery. This is an odd
 *   thing in our case because nothing else is using the SPI port, but
 *   that's not the general presentation so I'm staying consistent
 *   with the pattern.
 * 
 *                                      add by herc  2021.10.26
 *                                            copy from omnibusf4 board file
 *
 ****************************************************************************/

int stm32_mpu60x0_initialize(void)
{
    FAR struct i2c_master_s *i2cbus;
    int ret;

    int exti = GPIO_EXTI_MPU60X0;
    UNUSED(exti);
    stm32_configgpio(GPIO_EXTI_MPU60X0);
    
//    mpu_config = kmm_zalloc(sizeof(struct mpu_config_s));
//    if (mpu_config == NULL)
//    {
//        ferr("ERROR: Failed to allocate mpu60x0 driver\n");
//        return -ENOMEM;
//    }
//    mpu_config->i2c = i2cbus;
//    mpu_config->addr = 0x68;
  

    /* Get the i2c bus instance. */
    i2cbus = stm32_i2cbus_initialize(I2C_BUS_MPU60X0);
    if (!i2cbus)
    {
        ferr("ERROR: Failed to initialize I2CBUS%d\n", I2C_BUS_MPU60X0);
        return -ENODEV;
    }

    struct mpu_config_s mpu_config = 
    {
        .i2c  = i2cbus,
        .addr = 0x69,
    };

    /* TODO: configure EXTI pin */
    /* Register the chip with the device driver. */
    _alert("mpu60x0_register ：%s\n", DEVNODE_MPU60X0);
    ret = mpu60x0_register(DEVNODE_MPU60X0, &mpu_config);
    if (ret < 0)
    {
        ferr("ERROR: Failed to register I2C%d driver\n", I2C_BUS_MPU60X0);
        return -ENODEV;
    }

  return ret;
}
