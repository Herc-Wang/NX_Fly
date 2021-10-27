/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_pwm.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/timers/pwm.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "stm32f411-minimum.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/



/****************************************************************************
 * Name: stm32_mpu60x0_init
 *
 * Description:
 *   Initialize and configure the mpu60x0 
 *
 ****************************************************************************/

int stm32_mpu60x0_init(int minor)
{
  FAR struct i2c_master_s *i2c;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the I2C bus driver */
      _alert("Initialize I2C%d\n", MPU60X0_I2C_BUS);
      i2c = stm32_i2cbus_initialize(MPU60X0_I2C_BUS);
      if (!i2c)
        {
          ferr("ERROR: Failed to initialize I2C%d\n", MPU60X0_I2C_BUS);
          return -ENODEV;
        }

      _alert("register I2C%d\n", MPU60X0_I2C_BUS);
      ret = i2c_register(i2c, MPU60X0_I2C_BUS);
      if (ret < 0)
        {
          ferr("ERROR: Failed to register I2C%d driver\n", MPU60X0_I2C_BUS);
          return -ENODEV;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}