/****************************************************************************
 * boards/arm/rp2040/raspberrypi-pico-w/src/rp2040_boardinitialize.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "rp2040_gpio.h"
#include <nuttx/spi/spi.h>
#include "hardware/rp2040_spi.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp2040_common_initialize.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RFR_AUDIO_MUTE_PIN    17
#define RFR_AUDIO_SHND_PIN    16

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_boardearlyinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardearlyinitialize(void)
{
  #ifdef CONFIG_ARCH_BOARD_COMMON
  rp2040_common_earlyinitialize();
  #endif

  /* --- Place any board specific early initialization here --- */
}

/****************************************************************************
 * Name: rp2040_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardinitialize(void)
{
  #ifdef CONFIG_ARCH_BOARD_COMMON
  rp2040_common_initialize();
  #endif

  /* --- Place any board specific initialization here --- */

  /* Audio */

  rp2040_gpio_init(RFR_AUDIO_MUTE_PIN);
  rp2040_gpio_init(RFR_AUDIO_SHND_PIN);

  rp2040_gpio_setdir(RFR_AUDIO_MUTE_PIN, 1);
  rp2040_gpio_setdir(RFR_AUDIO_SHND_PIN, 1);

  rp2040_gpio_put(RFR_AUDIO_MUTE_PIN, 1);
  rp2040_gpio_put(RFR_AUDIO_SHND_PIN, 1);

}

void rp2040_spi0select(struct spi_dev_s *dev, uint32_t devid,
  bool selected)
{
  syslog(LOG_INFO, "CS OVERRIDEN!!!\n");

  rp2040_gpio_put(CONFIG_RP2040_SPI0_CS_GPIO, !selected);
}

int rp2040_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  syslog(LOG_INFO, "CMD OVERRIDEN!!!\n");

  rp2040_gpio_put(8, !cmd);

  return OK;
}