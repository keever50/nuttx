/****************************************************************************
 * include/nuttx/contactless/pn532.h
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

#ifndef __INCLUDE_NUTTX_CONTACTLESS_PN532_H
#define __INCLUDE_NUTTX_CONTACTLESS_PN532_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <sys/ioctl.h>

#include <nuttx/contactless/ioctl.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define PN532_NO_TIMEOUT                  (0x00)
#define PN532_MAX_FRAME_LENGTH            64
#define PN532_MIFARE_A_MAX_UID_LEN        7
#define PN532_MIFARE_B_MAX_ATTRIBUTE_LEN  12
#define PN532_MAX_CARDS_ONE_SCAN          2

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pn532_dev_s;
struct pn532_config_s
{
  int (*reset)(uint8_t enable);

  /* External CS, if NULL then SPIDEV_WIRELESS(n) CS is used */

  int (*select)(struct pn532_dev_s *dev, bool sel);
  int (*irqattach)(void *dev, xcpt_t isr);
};

enum pn532_sam_e
{
  PN532_SAM_NORMAL = 1,
  PN532_SAM_VIRTUAL,
  PN532_SAM_WIRED,
  PN532_SAM_DUAL
};

/* Cards */

/* This can also be used for card type */

enum pn532_baudmod_e
{
  PN532_BAUDMOD_TYPE_A_106K,
  PN532_BAUDMOD_FELICA_212K,
  PN532_BAUDMOD_FELICA_424K,
  PN532_BAUDMOD_TYPE_B_106K,
  PN532_BAUDMOD_JEWEL_106K
};

struct pn532_card_mifare_a_s
{
  uint8_t sens_res[2];
  uint8_t sel_res;
  uint8_t uid_len;
  uint8_t uid[PN532_MIFARE_A_MAX_UID_LEN];
};

struct pn532_card_mifare_b_s
{
  uint8_t atqb[12];
  uint8_t attrib_len;
  uint8_t attributes[PN532_MIFARE_B_MAX_ATTRIBUTE_LEN];
};

union pn532_card_info_u
{
  /* PN532_BAUDMOD_TYPE_A_106K */

  struct pn532_card_mifare_a_s mifare_a;

  /* PN532_BAUDMOD_TYPE_B_106K */

  struct pn532_card_mifare_b_s mifare_b;
};

struct pn532_card_s
{
  enum pn532_baudmod_e type;
  uint8_t target_nr;
  union pn532_card_info_u info;
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: pn532_register
 *
 * Description:
 *   Register the PN532 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/nfc0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             PN532
 *   config  - Device persistent board data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pn532_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                   FAR struct pn532_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CONTACTLESS_PN532_H */
