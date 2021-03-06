/****************************************************************************
 * boards/arm/stm32/stm32f411-minimum/src/stm32f411-minimum.h
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

#ifndef __BOARDS_ARM_STM32_STM32F411_MINIMUM_SRC_STM32F411_MINIMUM_H
#define __BOARDS_ARM_STM32_STM32F411_MINIMUM_SRC_STM32F411_MINIMUM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/chip/chip.h>  	 //add by herc
#include "stm32_gpio.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* LED.  User LED1: the blue LED is a user LED connected to board LED B8
 * corresponding to MCU I/O PB8
 *
 * - When the I/O is LOW value, the LED is on.
 * - When the I/O is HIGH, the LED is off.
 */

#define GPIO_LED1 \
  (GPIO_PORTB | GPIO_PIN8 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PULLUP | \
   GPIO_SPEED_50MHz)				//fix by herc

/* Buttons
 *
 * B1 USER: the user button is connected to the I/O PA0 of the STM32
 * microcontroller.
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER \
  (GPIO_INPUT |GPIO_FLOAT |GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)

/* SPI1 off */

#define GPIO_SPI1_MOSI_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN7)
#define GPIO_SPI1_MISO_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN6)
#define GPIO_SPI1_SCK_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTA | GPIO_PIN5)

/* NRF24L01
 * CS  - PB12 
 * CE  - PA8 
 * IRQ - PB2
 */

#define GPIO_NRF24L01_CS   (GPIO_OUTPUT | GPIO_SPEED_100MHz |           \
                            GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN12)
#define GPIO_NRF24L01_CE   (GPIO_OUTPUT | GPIO_SPEED_100MHz |             \
                            GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN8)
#define GPIO_NRF24L01_IRQ  (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN2)

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PC0  OTG_FS_PowerSwitchOn
 * PD5  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN0)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|\
                           GPIO_SPEED_100MHz|GPIO_PUSHPULL|\
                           GPIO_PORTD|GPIO_PIN5)

#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN5)
#endif


/* PWM   ----------herc
*
*/

/* PWM Configuration */

#define STM32F411MINIMUM_PWMTIMER   3
#define STM32F411MINIMUM_PWMCHANNEL 1


/* GPIO pins used by the GPIO Subsystem   ---add by  herc*/

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

//#define GPIO_IN1          (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN8)
//#define GPIO_OUT1         (GPIO_OUTPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN8)
//#define GPIO_INT1         (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN8)

#define GPIO_IN1          (GPIO_PORTB | GPIO_PIN9 | GPIO_INPUT | GPIO_SPEED_50MHz)
#define GPIO_OUT1         (GPIO_PORTB | GPIO_PIN8 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_50MHz)
#define GPIO_INT1         (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN10)



/* I2C   ----------herc
*
*/
#define I2C_BUS   1

/* I2c Configuration  this part finaly will define and set in the menuconfig, in other way ,set in Kconig*/
//#define CONFIG_STM32_I2CBUS_ID 1
//#define MPU60X0_I2C_BUS   1 /* MPU60X0 connected to I2C1 */
//#define MPU60X0_MINOR     1


/* MPU60X0   ----------herc */

//#define I2C_BUS_MPU60X0   1
#define I2C_MINOR_MPU60X0  0
#define GPIO_EXTI_MPU60X0 (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | \
                           GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN4)
#define DEVNODE_MPU60X0   "/dev/imu0"


/* SPL06   ----------herc  */
#define DEVNODE_SPL06     "/dev/spl0"

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32_SPI2
extern struct spi_dev_s *g_spi2;
#endif

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_boardinitialize very early in initialization to setup
 *   USB-related GPIO pins for the STM32F4Discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.  This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   PWM initialization               herc
 *
 ****************************************************************************/
#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio        herc
 *
 ****************************************************************************/
#ifdef CONFIG_DEV_GPIO
int stm32_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_mpu60x0_initialize
 *
 * Description:
 *   Initialize MPU60X0 drivers for use with /apps/examples/XX        herc
 *
 ****************************************************************************/
#ifdef CONFIG_SENSORS_MPU60X0
int stm32_mpu60x0_initialize(FAR struct i2c_master_s *i2cbus);
#endif

#ifdef CONFIG_SENSORS_SPL06
int stm32_spl06_initialize(FAR struct i2c_master_s *i2cbus);
#endif


/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __BOARDS_ARM_STM32_STM32F411_MINIMUM_SRC_STM32F411_MINIMUM_H */
