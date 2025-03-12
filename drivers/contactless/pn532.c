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
#include <nuttx/contactless/pn532.h>
#include <nuttx/spi/spi.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/power/pm.h>
#include <endian.h>
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

/* Commands */

static enum pn532_error_e pn532_send_command(FAR struct pn532_dev_s *dev,
                                             FAR uint8_t *params,
                                             size_t params_size,
                                             FAR uint8_t *answer,
                                             size_t answer_size,
                                             uint32_t timeout_ms);

static enum pn532_error_e pn532_samconfig(FAR struct pn532_dev_s *dev,
                                              enum pn532_sam_e mode,
                                              uint8_t timeout,
                                              uint8_t irq_enabled);


static enum pn532_error_e
pn532_in_list_pasv_tgt(FAR struct pn532_dev_s *dev,
                       uint8_t max_targets,
                       enum pn532_baudmod_e baudmod,
                       uint8_t *initiator_data,
                       size_t data_len,
                       uint32_t timeout_ms);

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

  /* Assume device is sleeping */

  dev->active = false;

  /* Hardware reset the device */

  dev->config->reset(true);
  nxsig_usleep(PN532_RESET_TIME_US);
  dev->config->reset(false);
  nxsig_usleep(PN532_RESET_TIME_US);

  /* Set default mode */

  enum pn532_error_e err;

  err = pn532_samconfig(dev, PN532_SAM_NORMAL,
                      PN532_NO_TIMEOUT,
                      0);

  if (err != PN532_OK)
    {
      return ERROR;
    }

  // TEST

  uint8_t in_data[] =
  {

  };
  err = pn532_in_list_pasv_tgt(dev, 2, PN532_BAUDMOD_TYPE_A_106K, in_data, sizeof(in_data), 10000);

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
      SPI_SELECT(dev->spi, SPIDEV_CONTACTLESS(PN532_SPI_ID), true);
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
      SPI_SELECT(dev->spi, SPIDEV_CONTACTLESS(PN532_SPI_ID), false);
    }
}

static void pn532_spi_start(FAR struct pn532_dev_s *dev)
{
  pn532_lock(dev->spi);
  pn532_select(dev);
}

static void pn532_spi_end(FAR struct pn532_dev_s *dev)
{
  pn532_deselect(dev);
  pn532_unlock(dev->spi);
}

/* SPI data control  ********************************************************/

/****************************************************************************
 * Name: pn532_spi_get_status
 *
 * Description:
 *   Polls the status of the device over SPI.
 *   This checks whether the device
 *   is busy or not through SPI communication.
 *
 ****************************************************************************/

static uint8_t pn532_spi_get_status(FAR struct pn532_dev_s *dev)
{
  /* ! This function requires a bus lock and select */

  pn532_spi_send(dev->spi, PN532_FR_STATREAD);
  uint8_t status = pn532_spi_send(dev->spi, 0x00);
  if (status & 0x01)
    {
      return PN532_OK;
    }

  return PN532_BUSY;
}

/****************************************************************************
 * Name: pn532_spi_get_ack
 *
 * Description:
 *   Gets an ack frame from the device over SPI and parses it.
 *   This will also verify the packet.
 *
 ****************************************************************************/

static enum pn532_error_e pn532_spi_get_ack(FAR struct pn532_dev_s *dev)
{
  /* ! This function requires a bus lock and select */

  enum pn532_error_e ret;

  /* Prepare */

  uint8_t ackframe[PN532_FR_ACKFRAME_LEN];

  /* Data read byte */

  pn532_spi_send(dev->spi, PN532_FR_DATAREAD);

  /* Receive */

  pn532_spi_recvblock(dev->spi, ackframe, PN532_FR_ACKFRAME_LEN);

  /* Parse */

  size_t i = 0;

  /* Make sure the packet look right */

  if (ackframe[i++] != PN532_FR_PREAMBLE)
    {
      ctlserr("Wrong preamble in ack frame");
      return PN532_UNEXPECTED;
    }

  if (ackframe[i++] != PN532_FR_STARTCODE1)
    {
      ctlserr("Wrong startcode in ack frame");
      return PN532_UNEXPECTED;
    }

  if (ackframe[i++] != PN532_FR_STARTCODE2)
    {
      ctlserr("Wrong startcode in ack frame");
      return PN532_UNEXPECTED;
    }

  /* Check for ACK or NACK */

  uint16_t ack = *(uint16_t *)(ackframe + i);
  ack = htobe16(ack);
  i = i + 2;

  if (ack == PN532_FR_ACK)
    {
      ret = PN532_OK;
    }
  else if (ack == PN532_FR_NACK)
    {
      ret = PN532_NACK;
    }
  else
    {
      ctlserr("Invalid ACK/NACK reponse");
      return PN532_UNEXPECTED;
    }

  /* Check if the last bit also looks right */

  if (ackframe[i] != PN532_FR_POSTAMBLE)
    {
      ctlserr("Wrong postamble in ack frame");
      return PN532_UNEXPECTED;
    }

  /* Return result OK/NACK */

  return ret;
}

/****************************************************************************
 * Name: pn532_spi_get_frame
 *
 * Description:
 *   Gets a normal frame from the device over SPI.
 *   Also parses and verifies the frame.
 *
 ****************************************************************************/

static enum pn532_error_e pn532_spi_get_frame(FAR struct pn532_dev_s *dev,
                                              FAR uint8_t *answer,
                                              uint8_t answer_len)
{
  /* ! This function requires a bus lock and select */

  /* Send read byte */

  pn532_spi_send(dev->spi, PN532_FR_DATAREAD);

  /* Get start. This contains length and checksum */

  uint8_t start[PN532_FR_INFOSTART_LEN];
  pn532_spi_recvblock(dev->spi, start, PN532_FR_INFOSTART_LEN);

  /* Make sure the packet look right */

  size_t i = 0;
  if (start[i++] != PN532_FR_PREAMBLE)
    {
      ctlserr("Wrong preamble in frame");
      return PN532_UNEXPECTED;
    }

  if (start[i++] != PN532_FR_STARTCODE1)
    {
      ctlserr("Wrong startcode in frame");
      return PN532_UNEXPECTED;
    }

  if (start[i++] != PN532_FR_STARTCODE2)
    {
      ctlserr("Wrong startcode in frame");
      return PN532_UNEXPECTED;
    }

  /* Get frame info */

  uint8_t len = start[i++];
  uint8_t len_cs = start[i++];
  uint8_t tfi = start[i++];

  /* Verify checksum */

  if ((uint8_t)(len + len_cs) != 0x00)
    {
      ctlserr("Frame length checksum fail");
      return PN532_CHECKSUM_FAIL;
    }

  /* check TFI the frame identifier */

  if (tfi != PN532_FR_PN532TOHOST)
    {
      ctlserr("Wrong frame identifier");
      return PN532_UNEXPECTED;
    }

  /* Does the data fit in the user buffer?
   * LEN includes the TFI byte. We already checked that.
   * This is not relevant for the answer returned.
   * Therefore len-1, removing the TFI from the equation.
   */

  if (len - 1 > answer_len)
    {
      ctlserr("Frame data does not fit in user buffer");
      return PN532_MEMORY;
    }

  /* Transfer data and calculate checksum */

  pn532_spi_recvblock(dev->spi, answer, len - 1);

  uint8_t new_cs = tfi;
  for (size_t d = 0; d < len - 1; d++)
    {
      uint8_t data = answer[d];
      new_cs = new_cs + data;
    }

  /* Get last bit of the packet */

  uint8_t last[PN532_FR_INFOSTART_REMAINDER_LEN];
  pn532_spi_recvblock(dev->spi, last,
              PN532_FR_INFOSTART_REMAINDER_LEN);

  /* Verify data */

  i = 0;
  uint8_t data_cs = last[i++];

  if ((uint8_t)(new_cs + data_cs) != 0x00)
    {
      ctlserr("Frame data checksum error");
      return PN532_CHECKSUM_FAIL;
    }

  /* Check last postamble */

  if (last[i] != PN532_FR_POSTAMBLE)
    {
      ctlserr("Wrong frame postamble");
      return PN532_UNEXPECTED;
    }

  return PN532_OK;
}

/****************************************************************************
 * Name: pn532_spi_send_frame
 *
 * Description:
 *   Sends a normal frame over SPI
 *
 ****************************************************************************/

static enum pn532_error_e pn532_spi_send_frame(FAR struct pn532_dev_s *dev,
                                               FAR uint8_t *params,
                                               size_t params_size)
{
  /* ! This function requires a bus lock and select */

  /* Prepare sending the frame */

  uint8_t length = 1 + params_size; /* TFI and PD0 to PDn */
  uint8_t length_cs = (uint8_t)(-(int)length);
  uint8_t data_cs = PN532_FR_HOSTTOPN532;
  for (size_t i = 0; i < params_size; i++)
    {
      data_cs = data_cs + params[i];
    }

  data_cs = (uint8_t)-(int)data_cs;

  uint8_t hostframe_start[] =
  {
    PN532_FR_DATAWRITE, /* Technically not part of frame */
    PN532_FR_PREAMBLE,
    PN532_FR_STARTCODE1,
    PN532_FR_STARTCODE2,
    length,
    length_cs,
    PN532_FR_HOSTTOPN532
  };

  uint8_t hostframe_end[] =
  {
    data_cs,
    PN532_FR_POSTAMBLE
  };

  /* Send out */

  /* OPTIMIZATION: find a way to send this all at once */

  pn532_spi_sndblock(dev->spi, hostframe_start, sizeof(hostframe_start));
  pn532_spi_sndblock(dev->spi, params, params_size);
  pn532_spi_sndblock(dev->spi, hostframe_end, sizeof(hostframe_end));

  return PN532_OK;
}

/* SPI Command control ******************************************************/

/****************************************************************************
 * Name: pn532_spi_wait_rdy
 *
 * Description:
 *   Wait (block) till the device is ready.
 *   In case of polling, over SPI.
 *
 ****************************************************************************/

static enum pn532_error_e pn532_spi_wait_rdy(FAR struct pn532_dev_s *dev,
                                             uint32_t timeout_ms,
                                             uint8_t critical)
{
  /* In case of no IRQ, do polling */

#if PN532_IRQ == 0

  clock_t timeout = clock_systime_ticks();
  while (clock_systime_ticks()-timeout < MSEC2TICK(timeout_ms))
    {
      /* Lock only during polling. Allows bus sharing */

      pn532_spi_start(dev);

      /* Poll */

      enum pn532_error_e ret = pn532_spi_get_status(dev);

      pn532_spi_end(dev);

      /* Device busy */

      if (ret == PN532_BUSY)
        {
          /* Wait a bit so others can use the bus */

          nxsig_usleep((PN532_POLLING_INTERVAL_MS * 1000));
          continue;
        }

      return PN532_OK;
    }

  if (critical)
    {
      ctlserr("Timed out waiting for ready status");
    }


  return PN532_TIMEOUT;

#elif
  /* TODO: Wait for IRQ line, here */

  /* Hint: Semaphore via hi prio worker */

  /* Hint: Return OK on ready, otherwise TIMEOUT */
#endif
}

/****************************************************************************
 * Name: pn532_spi_send_cmd
 *
 * Description:
 *   Sends a command over SPI to device.
 *   This contains the logic flow to control sending and receiving
 *   SPI frames. The bus can be shared between actions.
 *
 ****************************************************************************/

static enum pn532_error_e pn532_spi_send_cmd(FAR struct pn532_dev_s *dev,
                                      FAR uint8_t *params,
                                      size_t params_size,
                                      FAR uint8_t *answer,
                                      size_t answer_size,
                                      uint32_t timeout_ms)
{
  enum pn532_error_e ret = 0;

  /* Send command */

  pn532_spi_start(dev);

  /* Wake up the device by waiting a bit if sleeping */

  if (dev->active == false)
    {
      nxsig_usleep(PN532_WAKEUP_US);
      dev->active = true;
    }

  ret = pn532_spi_send_frame(dev, params, params_size);
  if (ret != PN532_OK)
    {
      goto frame_exit;
    }

  pn532_spi_end(dev);

  /* Wait for ready */

  ret = pn532_spi_wait_rdy(dev, PN532_TIMEOUT_MS_DEFAULT, true);
  if (ret != PN532_OK)
    {
      goto frame_exit;
    }

  /* Get acknowledge */

  pn532_spi_start(dev);

  ret = pn532_spi_get_ack(dev);
  if (ret == PN532_NACK)
    {
      ctlswarn("NACK on command");
      goto frame_exit;
    }
  else if (ret != PN532_OK)
    {
      goto frame_exit;
    }

  pn532_spi_end(dev);

  /* Wait for ready */

  ret = pn532_spi_wait_rdy(dev, timeout_ms, false);
  if (ret != PN532_OK)
    {
      goto frame_exit;
    }

  /* Get response */

  pn532_spi_start(dev);

  ret = pn532_spi_get_frame(dev, answer, answer_size);
  if (ret != PN532_OK)
    {
      goto frame_exit;
    }

frame_exit:

  /* OPTIONAL: add ACK/NACK response here. PN532 does not require it. */

  pn532_spi_end(dev);

  return ret;
}

/* Command control **********************************************************/

/****************************************************************************
 * Name: pn532_send_command
 *
 * Description:
 *   Sends a command over the configured interface.
 *   This is the top level function for controlling the device.
 *
 ****************************************************************************/

static enum pn532_error_e pn532_send_command(FAR struct pn532_dev_s *dev,
                                      FAR uint8_t *params,
                                      size_t params_size,
                                      FAR uint8_t *answer,
                                      size_t answer_size,
                                      uint32_t timeout_ms)
{
  enum pn532_error_e err;

  /* Other interface support can be added here */

  /* SPI interface */

#if PN532_IF_SPI == 1
  err = pn532_spi_send_cmd(dev, params,
                           params_size,
                           answer,
                           answer_size,
                           timeout_ms);
#endif

  return err;
}

/* Commands *****************************************************************/

/****************************************************************************
 * Name: pn532_samconfig
 *
 * Description:
 *   This command is used to select the data flow path by configuring
 *   the internal serial data switch.
 *
 ****************************************************************************/

static enum pn532_error_e pn532_samconfig(FAR struct pn532_dev_s *dev,
                                              enum pn532_sam_e mode,
                                              uint8_t timeout,
                                              uint8_t irq_enabled)
{
  enum pn532_error_e ret;

  uint8_t params[] =
  {
    PN532_COMMAND_SAMCONFIGURATION,
    mode,
    timeout,
    irq_enabled
  };

  uint8_t ans[1];

  ret = pn532_send_command(dev, params, sizeof(params),
             ans, sizeof(ans),
             PN532_TIMEOUT_MS_DEFAULT);

  return ret;
}

/****************************************************************************
 * Name: pn532_in_list_pasv_tgt
 *
 * Description:
 *   The goal of this command is to detect as max_targets
 *   as possible in passive mode.
 *
 ****************************************************************************/

static enum pn532_error_e
pn532_in_list_pasv_tgt(FAR struct pn532_dev_s *dev,
                       uint8_t max_targets,
                       enum pn532_baudmod_e baudmod,
                       uint8_t *initiator_data,
                       size_t data_len,
                       uint32_t timeout_ms)
{
  enum pn532_error_e ret;

  /* data and params cant be larger than the working buffer */

  size_t totalsize = PN532_COMMAND_INLISTPASSIVETARGET_PARAMS + data_len;
  if (totalsize > PN532_WORKBUFFER_SIZE)
    {
      ctlserr("Not enough space in working buffer for initiator data");
      return PN532_MEMORY;
    }

  uint8_t params[PN532_COMMAND_INLISTPASSIVETARGET_PARAMS] =
  {
    PN532_COMMAND_INLISTPASSIVETARGET,
    max_targets,
    (uint8_t)baudmod
  };

  /* Store params + data in working buffer */

  memcpy(dev->work_buffer, params, sizeof(params));
  memcpy(dev->work_buffer + sizeof(params), initiator_data, data_len);

  ret = pn532_send_command(dev, dev->work_buffer, totalsize,
                           dev->work_buffer, sizeof(dev->work_buffer),
                           timeout_ms);

  /* Get answer ... */

  ctlsinfo("Detection");

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pn532_register
 *
 * Description:
 *   Registers the device to a path. A good path name would look like
 *   /dev/rfid0, but anything can be used.
 *
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
