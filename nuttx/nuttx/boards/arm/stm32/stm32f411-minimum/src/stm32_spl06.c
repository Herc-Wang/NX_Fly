/****************************************************************************
 * boards/arm/stm32/stm32f411-minimum/src/stm32_spl06.c
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

#include <nuttx/sensors/spl06.h>
#include "stm32f411-minimum.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spl06_initialize
 *
 * Description:
 *
 *   Initialize and register's stm32f411-minimum's spl06. The wiring
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
 *                                            copy from stm32_mpu60x0.h
 *
 ****************************************************************************/

int stm32_spl06_initialize(FAR struct i2c_master_s *i2cbus)
{
    
    int ret; 

    struct spl_config_s spl_config = 
    {
        .i2c  = i2cbus,
        .addr = 0x77,     //SPL06_I2C_ADDR,
    };

    _alert("spl_register ：%s\n", DEVNODE_SPL06);
    ret = spl06_register(DEVNODE_SPL06, &spl_config);
    if (ret < 0)
    {
        ferr("ERROR: Failed to register spl06 driver\n");
        return -ENODEV;
    }

  return ret;
}
