/****************************************************************************
 * boards/arm/stm32/stm32f411-minimum/src/stm32_bringup.c
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

#include <debug.h>

#include <nuttx/fs/fs.h>

#include "stm32.h"

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#include "stm32f411-minimum.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;
//  long long leds_running_time = 0;
//  bool leds_status = true;

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
  /* Initialize USB host operation.  stm32_usbhost_initialize() starts a
   * thread will monitor for USB connection and disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif


#if defined(CONFIG_STM32_PWM)
  /* Initialize pwm -----herc
   */

  ret = stm32_pwm_setup();
//_alert("--alert------stm32_pwm_setup --------success----\r\n");
  if (ret != OK)
    {
      uerr("ERROR: Failed to initialize PWM: %d\n", ret);
      return ret;
    }
#endif


//  test userleds by autoleds' example    ---add by wjh
/*   
board_autoled_initialize();

while(leds_running_time < 500000)	
{
   leds_running_time++;
   if(leds_running_time%100000 == 0)
   {
	leds_status=!leds_status;
	set_led(leds_status);
   }
}
*/
#ifdef CONFIG_EXAMPLES_GPIO
  /* GPIO INIT */
  stm32_gpio_initialize();
_alert("--alert------stm32_gpio_initialize --------success----\r\n");
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount procfs at %s: %d\n",
           STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

  return ret;
}
