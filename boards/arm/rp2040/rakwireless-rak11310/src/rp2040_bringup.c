/****************************************************************************
 * boards/arm/rp2040/raspberrypi-pico/src/rp2040_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stddef.h>

#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include "rp2040_pico.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp2040_common_bringup.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

/* Custom driver */
#include "rp2040_spi.h"
#include <arch/board/rp2040_spidev.h>
#include <nuttx/spi/spi.h>
#include "rp2040_gpio.h"
#include <nuttx/wireless/lpwan/sx126x.h>

/****************************************************************************
 * Private prototypes
 ****************************************************************************/

void sx_reset(void);

/****************************************************************************
 * Private data
 ****************************************************************************/

struct sx126x_lower_s sx126x =
{
  .port=1,
  .reset=sx_reset
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

void sx_reset(void)
{
  rp2040_gpio_put(14, false);
  usleep(100);
  rp2040_gpio_put(14, true);
  usleep(100);  
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_bringup
 ****************************************************************************/

int rp2040_bringup(void)
{
#ifdef CONFIG_ARCH_BOARD_COMMON

  int ret = rp2040_common_bringup();
  if (ret < 0)
    {
      return ret;
    }

#endif /* CONFIG_ARCH_BOARD_COMMON */

  /* --- Place any board specific bringup code here --- */

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, \
      "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif
  
  rp2040_gpio_init(25);
  rp2040_gpio_setdir(25, true);
  rp2040_gpio_put(25, false);

  rp2040_gpio_init(14);
  rp2040_gpio_setdir(14, true);


  struct spi_dev_s *spi;
  spi=rp2040_spibus_initialize(sx126x.port);

  sx126x_register(spi, &sx126x);

  return OK;
}
