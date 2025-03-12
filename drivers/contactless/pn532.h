/****************************************************************************
 * drivers/contactless/pn532.h
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

#ifndef __DRIVERS_CONTACTLESS_PN532_H
#define __DRIVERS_CONTACTLESS_PN532_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>
#include <nuttx/contactless/pn532.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define PN532_IF_SPI  1
#define PN532_IRQ     0
#define PN532_SPI_ID  0

#define PN532_WORKBUFFER_SIZE               128

/* Timings ******************************************************************/

#define PN532_RESET_TIME_US                 10000
#define PN532_POLLING_INTERVAL_MS           0 /* Time between ready checks. 0 = none */
#define PN532_WAKEUP_US                     2000
#define PN532_TIMEOUT_MS_DEFAULT            1000

/* Frame ********************************************************************/

#define PN532_FR_PREAMBLE                      0x00
#define PN532_FR_STARTCODE1                    0x00
#define PN532_FR_STARTCODE2                    0xFF
#define PN532_FR_POSTAMBLE                     0x00

#define PN532_FR_HOSTTOPN532                   0xD4
#define PN532_FR_PN532TOHOST                   0xD5

#define PN532_FR_STATREAD                      0x02
#define PN532_FR_DATAWRITE                     0x01
#define PN532_FR_DATAREAD                      0x03
#define PN532_FR_READY                         0x01

#define PN532_FR_ACK                           0x00FF
#define PN532_FR_NACK                          0xFF00

#define PN532_FR_INFOSTART_LEN                 6
#define PN532_FR_INFOSTART_REMAINDER_LEN       2
#define PN532_FR_ACKFRAME_LEN                  6

/* PN532 Commands ***********************************************************/

#define PN532_COMMAND_DIAGNOSE              0x00
#define PN532_COMMAND_GETFIRMWAREVERSION    0x02
#define PN532_COMMAND_GETGENERALSTATUS      0x04
#define PN532_COMMAND_READREGISTER          0x06
#define PN532_COMMAND_WRITEREGISTER         0x08
#define PN532_COMMAND_READGPIO              0x0C
#define PN532_COMMAND_WRITEGPIO             0x0E
#define PN532_COMMAND_SETSERIALBAUDRATE     0x10
#define PN532_COMMAND_SETPARAMETERS         0x12
#define PN532_COMMAND_SAMCONFIGURATION      0x14
#define PN532_COMMAND_POWERDOWN             0x16
#define PN532_COMMAND_RFCONFIGURATION       0x32
#define PN532_COMMAND_RFREGULATIONTEST      0x58
#define PN532_COMMAND_INJUMPFORDEP          0x56
#define PN532_COMMAND_INJUMPFORPSL          0x46
#define PN532_COMMAND_INLISTPASSIVETARGET   0x4A
#define PN532_COMMAND_INATR                 0x50
#define PN532_COMMAND_INPSL                 0x4E
#define PN532_COMMAND_INDATAEXCHANGE        0x40
#define PN532_COMMAND_INCOMMUNICATETHRU     0x42
#define PN532_COMMAND_INDESELECT            0x44
#define PN532_COMMAND_INRELEASE             0x52
#define PN532_COMMAND_INSELECT              0x54
#define PN532_COMMAND_INAUTOPOLL            0x60
#define PN532_COMMAND_TGINITASTARGET        0x8C
#define PN532_COMMAND_TGSETGENERALBYTES     0x92
#define PN532_COMMAND_TGGETDATA             0x86
#define PN532_COMMAND_TGSETDATA             0x8E
#define PN532_COMMAND_TGSETMETADATA         0x94
#define PN532_COMMAND_TGGETINITIATORCOMMAND 0x88
#define PN532_COMMAND_TGRESPONSETOINITIATOR 0x90
#define PN532_COMMAND_TGGETTARGETSTATUS     0x8A

#define PN532_WAKEUP                        0x55

/* Param lengths */

#define PN532_COMMAND_INLISTPASSIVETARGET_PARAMS 3

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pn532_dev_s
{
  FAR struct spi_dev_s *spi;          /* SPI interface */
  FAR struct pn532_config_s *config;  /* Board configuration data */
  uint8_t active;
  uint8_t work_buffer[PN532_WORKBUFFER_SIZE];
};

enum pn532_error_e
{
  PN532_OK,
  PN532_TIMEOUT,
  PN532_NACK,
  PN532_CHECKSUM_FAIL,
  PN532_BUSY,
  PN532_UNEXPECTED,
  PN532_MEMORY
};

enum pn532_baudmod_e
{
  PN532_BAUDMOD_TYPE_A_106K,
  PN532_BAUDMOD_FELICA_212K,
  PN532_BAUDMOD_FELICA_424K,
  PN532_BAUDMOD_TYPE_B_106K,
  PN532_BAUDMOD_JEWEL_106K
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __DRIVERS_CONTACTLESS_PN532_H */
