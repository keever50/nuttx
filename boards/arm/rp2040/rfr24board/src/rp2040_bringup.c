/****************************************************************************
 * boards/arm/rp2040/raspberrypi-pico-w/src/rp2040_bringup.c
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
#include <errno.h>

#include <nuttx/wireless/ieee80211/bcmf_gpio.h>
#include "rp2040_spi.h"
#include <arch/board/rp2040_spidev.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7567.h>
#include <unistd.h>
#include "rp2040_gpio.h"
#include "rp2040_pico.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp2040_common_bringup.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

#ifdef CONFIG_RP2040_INFINEON_CYW43439
#include "rp2040_cyw43439.h"
#endif

/****************************************************************************
 * Global Data
 ****************************************************************************/

#ifdef CONFIG_RP2040_INFINEON_CYW43439
gspi_dev_t *g_cyw43439 = NULL;
#endif

FAR struct spi_dev_s *g_spi_dev0;
FAR struct lcd_dev_s *g_lcddev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_lcd_initialize(void)
{
  /* DC */

  rp2040_gpio_init(8);
  rp2040_gpio_setdir(8, 1);
  rp2040_gpio_put(8, 1);

  /* RESET */

  rp2040_gpio_init(9);
  rp2040_gpio_setdir(9, 1);
  rp2040_gpio_put(9, 0);

  usleep(10000);

  rp2040_gpio_put(9, 1);

  g_spi_dev0 = rp2040_spibus_initialize(0);
  //g_lcddev = st7567_initialize(g_spi_dev0, 0);

  return OK;
}

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  /* BL */

  rp2040_gpio_init(28);
  rp2040_gpio_setdir(28, 1);
  rp2040_gpio_put(28, 1);

  g_lcddev = st7567_initialize(g_spi_dev0, lcddev);
  if (!g_lcddev)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI port 1 to LCD %d\n", lcddev);
    }
  else
    {
      syslog(LOG_INFO, "SPI port 1 bound to LCD %d\n", lcddev);

      /* And turn the LCD on (CONFIG_LCD_MAXPOWER should be 1) */

      g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);

      /* Set contrast to right value, otherwise background too dark */

      g_lcddev->setcontrast(g_lcddev, 0x1f);

      return g_lcddev;
    }

  return NULL;
}


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

#define CYW43439_POWER_ON_GPIO     23
#define CYW43439_CHIP_SELECT_GPIO  25
#define CYW43439_DATA_GPIO         24
#define CYW43439_CLOCK_GPIO        29

#ifdef CONFIG_RP2040_INFINEON_CYW43439

  g_cyw43439 = rp2040_cyw_setup(CYW43439_POWER_ON_GPIO,
                                CYW43439_CHIP_SELECT_GPIO,
                                CYW43439_DATA_GPIO,
                                CYW43439_CLOCK_GPIO,
                                CYW43439_DATA_GPIO);

  if (g_cyw43439 == NULL)
    {
      ret = errno;

      syslog(LOG_ERR,
             "Failed to initialize cyw43439 (WiFi chip): %d\n",
             ret);

      return ret;
    }

#endif

  
  


  return OK;
}
