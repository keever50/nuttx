/****************************************************************************
 * include/nuttx/wireless/ioctl.h
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

/* This file includes common definitions to be used in all wireless
 * character drivers (when applicable).
 */

#ifndef __INCLUDE_NUTTX_WIRELESS_IOCTL_H
#define __INCLUDE_NUTTX_WIRELESS_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <fixedmath.h>

#ifdef CONFIG_DRIVERS_WIRELESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Character Driver IOCTL commands
 * Non-compatible, NuttX only IOCTL definitions for use with low-level
 * wireless drivers that are accessed via a character device.
 * Use of these IOCTL commands requires a file descriptor created by
 * the open() interface.
 ****************************************************************************/

/****************************************************************************
 * RF common IOCTL commands
 ****************************************************************************/

#define WLIOC_SETRADIOFREQ  _WLCIOC(0x0001)  /* arg: Pointer to uint32_t, */
                                             /* frequency value (in MHz) */
#define WLIOC_GETRADIOFREQ  _WLCIOC(0x0002)  /* arg: Pointer to uint32_t, */
                                             /* frequency value (in MHz) */
#define WLIOC_SETADDR       _WLCIOC(0x0003)  /* arg: Pointer to address value, format
                                              * of the address is driver specific */
#define WLIOC_GETADDR       _WLCIOC(0x0004)  /* arg: Pointer to address value, format
                                              * of the address is driver specific */
#define WLIOC_SETTXPOWER    _WLCIOC(0x0005)  /* arg: Pointer to int32_t, */
                                             /* output power (in 0.01 dBm) */
#define WLIOC_GETTXPOWER    _WLCIOC(0x0006)  /* arg: Pointer to int32_t, */
                                             /* output power (in 0.01 dBm) */

/* WARNING: The following WLIOC command are EXPERIMENTAL and unstable. They
 * may be removed or modified in upcoming changes that introduce a common
 * LoRa API. These commands are currently only used by the RN2XX3 driver.
 */

#define WLIOC_SETBANDWIDTH  _WLCIOC(0x0007)  /* arg: Pointer to uint32_t, */
                                             /* bandwidth in Hz */
#define WLIOC_GETBANDWIDTH  _WLCIOC(0x0008)  /* arg: Pointer to uint32_t, */
                                             /* bandwidth in Hz */
#define WLIOC_SETSPREAD     _WLCIOC(0x0009)  /* arg: Pointer to uint8_t, */
                                             /* spread factor */
#define WLIOC_GETSPREAD     _WLCIOC(0x000a)  /* arg: Pointer to uint8_t, */
                                             /* spread factor */
#define WLIOC_GETSNR        _WLCIOC(0x000b)  /* arg: Pointer to int8_t, */
                                             /* signal to noise ratio */
#define WLIOC_SETPRLEN      _WLCIOC(0x000c)  /* arg: uint16_t, */
                                             /* preamble length */
#define WLIOC_GETPRLEN      _WLCIOC(0x000d)  /* arg: Pointer to uint16_t, */
                                             /* preamble length */
#define WLIOC_SETMOD        _WLCIOC(0x000e)  /* arg: enum, */
                                             /* modulation type */
#define WLIOC_GETMOD        _WLCIOC(0x000f)  /* arg: enum pointer, */
                                             /* modulation type */
#define WLIOC_RESET         _WLCIOC(0x0010)  /* arg: none */
#define WLIOC_SETSYNC       _WLCIOC(0x0011)  /* arg: uint64_t pointer */
                                             /* sync word */
#define WLIOC_GETSYNC       _WLCIOC(0x0012)  /* arg: uint64_t pointer, */
                                             /* sync word */
#define WLIOC_SETBITRATE    _WLCIOC(0x0013)  /* arg: uint32_t */
                                             /* sync word */
#define WLIOC_GETBITRATE    _WLCIOC(0x0014)  /* arg: uint32_t pointer, */
                                             /* sync word */
#define WLIOC_IQIEN         _WLCIOC(0x0015)  /* arg: bool, enable invert IQ */
#define WLIOC_CRCEN         _WLCIOC(0x0016)  /* arg: bool, enable CRC */
#define WLIOC_SETCODERATE   _WLCIOC(0x0017)  /* arg: enum, coding rate */
#define WLIOC_GETCODERATE   _WLCIOC(0x0018)  /* arg: enum pointer, */
                                             /* coding rate */
#define WLIOC_SETTXPOWERF   _WLCIOC(0x0019)  /* arg: Pointer to float, */
                                             /* output power (in dBm) */
#define WLIOC_GETTXPOWERF   _WLCIOC(0x001a)  /* arg: Pointer to float, */
                                             /* output power (in dBm) */

/****************************************************************************
 * LoRa common IOCTL commands (EXPERIMENTAL)
 ****************************************************************************/

#define WLIOC_LORA_SETSF       _WLCIOC(0x001b) /* arg: Pointer to uint8_t */
                                               /* Spreading factor */
#define WLIOC_LORA_SETBW       _WLCIOC(0x001c) /* arg: Pointer to uint16_t */
                                               /* Bandwidth kHz */
#define WLIOC_LORA_SETCR       _WLCIOC(0x001d) /* arg: Pointer to wlioc_lora_cr_e */
                                               /* Coding rate */
#define WLIOC_LORA_ENCRC       _WLCIOC(0x001e) /* arg: Pointer to uint8_t */ 
                                               /* Enable/disable CRC */
#define WLIOC_LORA_ENFIXEDHDR  _WLCIOC(0x001f) /* arg: Pointer to uint8_t */
                                               /* Enable/disable length byte */
#define WLIOC_LORA_SYNCWORD    _WLCIOC(0x0020) /* arg: Pointer to wlioc_lora_syncword_s */
                                               /* Sets custom length syncword */

/****************************************************************************
 * Device-specific IOCTL commands
 ****************************************************************************/

#define WL_FIRST            0x0001          /* First common command */
#define WL_NCMDS            0x0020          /* Number of common commands */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half driver to the lower-half driver via the ioctl()
 * method of the lower-half interface.  However, the lower-half driver
 * must reserve a block of commands as follows in order prevent IOCTL
 * command numbers from overlapping.
 */

/* See include/nuttx/wireless/nrf24l01.h */

#define NRF24L01_FIRST      (WL_FIRST + WL_NCMDS)
#define NRF24L01_NCMDS      16

/* See include/nuttx/wireless/lpwan/sx127x.h */

#define SX127X_FIRST        (NRF24L01_FIRST + NRF24L01_NCMDS)
#define SX127X_NCMDS        11

/* See include/nuttx/wireless/lpwan/sx126x.h */

#define SX126X_FIRST        (SX127X_FIRST + SX127X_NCMDS)
#define SX126X_NCMDS        11

/* See include/nuttx/wireless/gs2200m.h */

#define GS2200M_FIRST       (SX127X_FIRST + SX127X_NCMDS)
#define GS2200M_NCMDS       9

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* LoRa common types ********************************************************/

/* wlioc_lora_cr_e is the coding rate, which is commonly
 * supported by LoRa devices to correct errors.
 */

enum wlioc_lora_cr_e
{
  WLIOC_LORA_CR_4_5 = 0x01,
  WLIOC_LORA_CR_4_6 = 0x02,
  WLIOC_LORA_CR_4_7 = 0x03,
  WLIOC_LORA_CR_4_8 = 0x04
};

/* wlioc_lora_syncword_s is used to seperate lora networks.
 * Radios will try to catch only packets with the specified syncword.
 */

struct wlioc_lora_syncword_s
{
  size_t syncword_length;
  uint8_t *syncword;
};

/* Common RF types **********************************************************/

/* wlioc_rx_hdr_s gets written to by reading the character device.
 * Contains information about the payload.
 */

struct wlioc_rx_hdr_s
{
  /* Length of payload in bytes.
   * The amount written to the
   * payload buffer
   */

  size_t payload_length;

  /* Pointer to user buffer
   * This will be filled in with
   * the payload
   */

  uint8_t *payload;

  /* When error detection is supported and enabled,
   * this will be greater than 0 when an error is
   * detected. The payload is still returned which
   * allows the user to reconstruct the payload
   */

  uint8_t error;

  /* RSSI dBm in 16 fractional bits fixed point. */

  b16_t rssi_dbm;

  /* SNR dB in 16 fractional bits fixed point.  */

  b16_t snr_db;
};

#endif /* CONFIG_DRIVERS_WIRELESS */
#endif /* __INCLUDE_NUTTX_WIRELESS_IOCTL_H */
