#ifndef __INCLUDE_NUTTX_SENSORS_SPL06_H
#define __INCLUDE_NUTTX_SENSORS_SPL06_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These structures are defined elsewhere, and we don't need their
 * definitions here.
 */


struct i2c_master_s;




/* Specifies the initial chip configuration and location.
 *
 * Important note :
 *
 * The driver determines which interface type to use according to
 * which of the two groups of fields is non-NULL.  Since support for
 * I2C is individually configurable, however, users should
 * let the compiler clear unused fields instead of setting unused
 * fields to NULL directly. For example, if using i2c and a
 * stack-allocated instance:
 *
 *   if using dynamic memory allocation and I2C:
 *
 *    struct mpu_config_s* splc;
 *    splc = kmm_malloc(sizeof(*splc));
 *    memset(splc, 0, sizeof(*splc)); * sets spi to NULL, if present *
 *    splc.i2c = ...;
 *
 * The above examples will avoid compile-time errors unless the user
 * forgets to enable their preferred interface type, and will allow
 * them to disable or enable the unused interface type without
 * changing their code.
 *
 */

struct spl_config_s
{
  /* For users on I2C.
   *
   *  i2c  : the I2C master device
   *  addr : the I2C address.
   */

  FAR struct i2c_master_s *i2c;
  int addr;

};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Declares the existence of an spl06 chip, wired according to
 * config; creates an interface to it at path.
 *
 * Returns 0 on success, or negative errno.
 */

int spl06_register(FAR const char *path, FAR struct spl_config_s *config);

#endif /* __INCLUDE_NUTTX_SENSORS_SPL06_H */