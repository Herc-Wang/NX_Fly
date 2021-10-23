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
#include "stm32_pwm.h"
#include "stm32f411-minimum.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PWM
 *	PWM1---PA6	TIM3_CH1
 *	PWM2---PA7	TIM3_CH2
 *	PWM3---PB0	TIM3_CH3
 *	PWM4---PB1	TIM3_CH4
 *
 */

#define HAVE_PWM 1

#ifndef CONFIG_PWM
#  undef HAVE_PWM
#endif

#ifndef CONFIG_STM32_TIM3
#  undef HAVE_PWM
#endif

#ifndef CONFIG_STM32_TIM3_PWM
#  undef HAVE_PWM
#endif

#if !defined(CONFIG_STM32_TIM3_CHANNEL)
#  undef HAVE_PWM
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

int stm32_pwm_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call stm32_pwminitialize() to get an instance of the PWM interface */
      pwm = stm32_pwminitialize(STM32F411MINIMUM_PWMTIMER);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm0" */

      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
_alert("--alert------stm32_pwm_setup ->pwm_register -------success----\r\n");

      /* Now we are initialized */

      initialized = true;
    }

  return OK;

}
