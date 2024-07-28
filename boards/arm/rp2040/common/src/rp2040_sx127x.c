/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_ssd1306.c
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
#include <nuttx/wireless/lpwan/sx127x.h>
#include <nuttx/spi/spi.h>
#include <nuttx/signal.h>

#include "rp2040_spi.h" /* probably has to be ifdeffed */
#include "rp2040_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OLED_I2C_PORT 0 /* OLED display connected to I2C0 */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sx127x_chip_reset(void);
static int sx127x_opmode_change(int opmode);
static int sx127x_freq_select(uint32_t freq);
static int sx127x_pa_select(bool enable);
static int sx127x_irq0_attach(xcpt_t isr, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct sx127x_lower_s lower =
    {
        .irq0attach = sx127x_irq0_attach,
        .reset = sx127x_chip_reset,
        .opmode_change = sx127x_opmode_change,
        .freq_select = sx127x_freq_select,
        .pa_select = sx127x_pa_select,
        .pa_force = true};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx127x_irq0_attach
 ****************************************************************************/

static int sx127x_irq0_attach(xcpt_t isr, void *arg)
{
  return ERROR;
}

/****************************************************************************
 * Name: sx127x_chip_reset
 ****************************************************************************/

static void sx127x_chip_reset(void)
{
  wlinfo("SX127X RESET\n");

  /* Configure reset as output */

  rp2040_gpio_set_function(20, RP2040_GPIO_FUNC_SIO);

  /* Set pin to zero */

  rp2040_gpio_put(20, 0);

  /* Wait 1 ms */

  nxsig_usleep(1000);

  /* Set pin to high */

  rp2040_gpio_put(20, 1);

  /* Wait 10 ms */

  nxsig_usleep(10000);
}

/****************************************************************************
 * Name: sx127x_opmode_change
 ****************************************************************************/

static int sx127x_opmode_change(int opmode)
{
  /* Do nothing */

  return OK;
}

/****************************************************************************
 * Name: sx127x_freq_select
 ****************************************************************************/

static int sx127x_freq_select(uint32_t freq)
{

  return OK;
}

/****************************************************************************
 * Name: sx127x_pa_select
 ****************************************************************************/

static int sx127x_pa_select(bool enable)
{
  int ret = OK;

  /* Only PA_BOOST output connected to antenna */

  if (enable == false)
  {
    ret = -EINVAL;
    wlerr("Module supports only PA_BOOST pin,"
          " so PA_SELECT must be enabled!\n");
  }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sx127x_initialize
 ****************************************************************************/

int board_sx127x_initialize(void)
{
  struct spi_dev_s *spidev;
  int ret = OK;

  wlinfo("Register the sx127x module\n");

  /* Setup DIO0 */

  // esp32_configgpio(GPIO_SX127X_DIO0, INPUT_FUNCTION_3 | PULLDOWN);

  /* Init SPI bus */

  spidev = rp2040_spibus_initialize(0);
  if (!spidev)
  {
    wlerr("ERROR: Failed to initialize SPI %d bus\n", 0);
    ret = -ENODEV;
    goto errout;
  }

  /* Initialize SX127X */

  ret = sx127x_register(spidev, &lower);
  if (ret < 0)
  {
    wlerr("ERROR: Failed to register sx127x\n");
    goto errout;
  }

errout:
  return ret;
}
