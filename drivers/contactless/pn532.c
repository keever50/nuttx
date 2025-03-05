/****************************************************************************
 * drivers/contactless/pn532.c
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
#include <assert.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/power/pm.h>

#include "pn532.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_CL_PN532_DEBUG_TX
#  define tracetx errdumpbuffer
#else
#    define tracetx(x...)
#endif

#ifdef CONFIG_CL_PN532_DEBUG_RX
#  define tracerx errdumpbuffer
#else
#    define tracerx(x...)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI control */

static inline void pn532_configspi(FAR struct spi_dev_s *spi);
static void        pn532_lock(FAR struct spi_dev_s *spi);
static void        pn532_unlock(FAR struct spi_dev_s *spi);

/* SPI wrappers */

uint8_t pn532_spi_reverse_bitorder(uint8_t b);
int     pn532_spi_send(FAR struct spi_dev_s *spi, uint8_t byte);
void    pn532_spi_recvblock(FAR struct spi_dev_s *spi, uint8_t *buffer,
                            size_t length);
void    pn532_spi_sndblock(FAR struct spi_dev_s *spi, uint8_t *buffer,
                           size_t length);

/* Character driver methods */

static int     pn532_open(FAR struct file *filep);
static int     pn532_close(FAR struct file *filep);
static ssize_t pn532_read(FAR struct file *filep,
                          FAR char *buffer, size_t buflen);
static ssize_t pn532_write(FAR struct file *filep,
                           FAR const char *buffer,
                           size_t buflen);
static int     pn532_ioctl(FAR struct file *filep,
                           int cmd, unsigned long arg);

/* Utility */

static uint8_t pn532_checksum(FAR uint8_t *data, int datalen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pn532fops =
{
  pn532_open,
  pn532_close,
  pn532_read,
  pn532_write,
  NULL, /* Seek */
  pn532_ioctl
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* File operations **********************************************************/

static int pn532_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct pn532_dev_s *dev;

  inode = filep->f_inode;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  /* Hardware reset the device */

  dev->config->reset(true);
  nxsig_usleep(PN532_RESET_TIME_US);
  dev->config->reset(false);

  return OK;
}

static int pn532_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct pn532_dev_s *dev;

  inode = filep->f_inode;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  UNUSED(dev);

  return OK;
}

static ssize_t pn532_read(FAR struct file *filep,
                          FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode;
  FAR struct pn532_dev_s *dev;

  inode = filep->f_inode;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  UNUSED(dev);

  return -EIO;
}

static ssize_t pn532_write(FAR struct file *filep, FAR const char *buffer,
                      size_t buflen)
{
  FAR struct inode *inode;
  FAR struct pn532_dev_s *dev;

  inode = filep->f_inode;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  UNUSED(dev);

  return -ENOSYS;
}

static int pn532_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct pn532_dev_s *dev;
  int ret = OK;

  inode = filep->f_inode;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  switch (cmd)
    {
    case PN532IOC_READ_TAG_DATA:
      break;

    case PN532IOC_WRITE_TAG_DATA:
      break;

    case PN532IOC_SET_SAM_CONF:
      break;

    case PN532IOC_READ_PASSIVE:
      break;

    case PN532IOC_SET_RF_CONF:
      break;

    case PN532IOC_SEND_CMD_READ_PASSIVE:
      break;

    case PN532IOC_GET_DATA_READY:
      break;

    case PN532IOC_GET_TAG_ID:
      break;

    case PN532IOC_GET_STATE:
      break;

    default:
      ctlserr("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/* SPI wrappers *************************************************************/

uint8_t pn532_spi_reverse_bitorder(uint8_t b)
{
  b = (b & 0xf0) >> 4 | (b & 0x0f) << 4;
  b = (b & 0xcc) >> 2 | (b & 0x33) << 2;
  b = (b & 0xaa) >> 1 | (b & 0x55) << 1;
  return b;
}

int pn532_spi_send(FAR struct spi_dev_s *spi, uint8_t byte)
{
  /* Correct bit order using software */

#ifndef CL_PN532_HW_BITORDER
  byte = pn532_spi_reverse_bitorder(byte);
#endif

  uint8_t ans = SPI_SEND(spi, byte);

  /* Once again, but for response */

#ifndef CL_PN532_HW_BITORDER  
  ans = pn532_spi_reverse_bitorder(ans);
#endif

  return ans;
}

void pn532_spi_recvblock(FAR struct spi_dev_s *spi, uint8_t *buffer,
                         size_t length)
{
  /* Correct bit order using software */

#ifndef CL_PN532_HW_BITORDER  
  for (size_t i = 0; i < length; i++)
    {
      buffer[i] = pn532_spi_reverse_bitorder(buffer[i]);
    }
#endif

  SPI_RECVBLOCK(spi, buffer, length);

  /* Once again, but for response */

#ifndef CL_PN532_HW_BITORDER  
  for (size_t i = 0; i < length; i++)
    {
      buffer[i] = pn532_spi_reverse_bitorder(buffer[i]);
    }
#endif
}

void pn532_spi_sndblock(FAR struct spi_dev_s *spi, uint8_t *buffer,
                        size_t length)
{
  /* Correct bit order using software */

#ifndef CL_PN532_HW_BITORDER  
  for (size_t i = 0; i < length; i++)
    {
      buffer[i] = pn532_spi_reverse_bitorder(buffer[i]);
    }
#endif

  SPI_SNDBLOCK(spi, buffer, length);

  /* Once again, but for response */

#ifndef CL_PN532_HW_BITORDER  
  for (size_t i = 0; i < length; i++)
    {
      buffer[i] = pn532_spi_reverse_bitorder(buffer[i]);
    }
#endif
}

/* SPI control  *************************************************************/

static void pn532_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the PN532 module. */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, CONFIG_PN532_SPI_FREQ);

  /* Use hardware bit order if possible
   * Not all architectures support this.
   */

#ifdef CL_PN532_HW_BITORDER
  SPI_HWFEATURES(spi, HWFEAT_LSBFIRST);
#endif
}

static void pn532_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);

  /* Reconfigure in case the bus is shared
   * with different settings.
   */

  pn532_configspi(spi);
}

static void pn532_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

static inline void pn532_select(FAR struct pn532_dev_s *dev)
{
  /* If alternative chip select is available,
   * use this over SPI chip select.
   */

  if (dev->config->select)
    {
      dev->config->select(dev, true);
    }
  else
    {
      SPI_SELECT(dev->spi, SPIDEV_CONTACTLESS(0), true);
    }
}

static inline void pn532_deselect(FAR struct pn532_dev_s *dev)
{
  /* If alternative chip select is available,
   * use this over SPI chip select.
   */

  if (dev->config->select)
    {
      dev->config->select(dev, false);
    }
  else
    {
      SPI_SELECT(dev->spi, SPIDEV_CONTACTLESS(0), false);
    }
}

/* Utility ******************************************************************/

static uint8_t pn532_checksum(FAR uint8_t *data, int datalen)
{
  uint8_t sum = 0x00;
  int i;

  for (i = 0; i < datalen; i++)
    {
      sum += data[i];
    }

  return sum;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pn532_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                   FAR struct pn532_config_s *config)
{
  FAR struct pn532_dev_s *dev;
  int ret;

  /* Initialize the PN532 device structure */

  dev = kmm_malloc(sizeof(struct pn532_dev_s));
  if (!dev)
    {
      ctlserr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  dev->spi = spi;
  dev->config = config;

  /* Register the character driver */

  ret = register_driver(devpath, &g_pn532fops, 0666, dev);
  if (ret < 0)
    {
      ctlserr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(dev);
    }

  return ret;
}
