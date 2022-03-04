/****************************************************************************
 * libs/libc/math/lib_atan2.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong
 *
 * It derives from the Rhombus OS math library by Nick Johnson which has
 * a compatibile, MIT-style license:
 *
 * Copyright (C) 2009-2011 Nick Johnson <nickbjohnson4224 at gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <math.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_DOUBLE
double atan2(double y, double x)
{
  double t;
  if (x > 0)
    {
      t = atan(y / x);
      //printf("*******************1***atan2 =  %lf\n", t);
      return t;
    }
  else if (y >= 0 && x < 0)
    {
      t = atan(y / x) + M_PI;
      //printf("*******************2***atan2 =  %lf\n", t);
      return t;
    }
  else if (y < 0)
    {
      if (x == 0)
        {
          t = -M_PI_2;
          //printf("****************3******atan2 =  %lf\n", t);
          return t;
        }
      else /* Can only be x < 0 */
        {
          t = atan(y / x) - M_PI;
          //printf("****************4******atan2 =  %lf\n", t);
          return t;
        }
    }
  else if (y > 0 && x == 0)
    {
      t = M_PI_2;
      //printf("******************5****atan2 =  %lf\n", t);
      return t;
    }
  else /* if (y == 0 && x == 0) Undefined but returns normally 0 */
    {
      //printf("****************6******atan2 =  0\n");
      return 0;
    }
}

double atan2_MY(double y, double x, double * result)
{
  if (x > 0)
    {
      * result = atan(y / x);
      //printf("*******************1***atan2 =  %lf\n", t);
      return 0;
    }
  else if (y >= 0 && x < 0)
    {
      * result = atan(y / x) + M_PI;
      //printf("*******************2***atan2 =  %lf\n", t);
      return 0;
    }
  else if (y < 0)
    {
      if (x == 0)
        {
          * result = -M_PI_2;
          //printf("****************3******atan2 =  %lf\n", t);
          return 0;
        }
      else /* Can only be x < 0 */
        {
          * result = atan(y / x) - M_PI;
          //printf("****************4******atan2 =  %lf\n", t);
          return 0;
        }
    }
  else if (y > 0 && x == 0)
    {
      * result = M_PI_2;
      //printf("******************5****atan2 =  %lf\n", t);
      return 0;
    }
  else /* if (y == 0 && x == 0) Undefined but returns normally 0 */
    {
      //printf("****************6******atan2 =  0\n");
      return 1;
    }
}
#endif
