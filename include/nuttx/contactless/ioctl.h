/****************************************************************************
 * include/nuttx/contactless/ioctl.h
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

#ifndef __INCLUDE_NUTTX_CONTACTLESS_IOCTL_H
#define __INCLUDE_NUTTX_CONTACTLESS_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* MFRC522 IOCTL Commands ***************************************************/

#define MFRC522IOC_GET_PICC_UID         _CLIOC(0x0001)
#define MFRC522IOC_GET_STATE            _CLIOC(0x0002)

/* PN532 IOCTL Commands *****************************************************/

#define PN532IOC_SET_SAM_CONF           _CLIOC(0x0003)
#define PN532IOC_READ_PASSIVE           _CLIOC(0x0004)
#define PN532IOC_SET_RF_CONF            _CLIOC(0x0005)
#define PN532IOC_SEND_CMD_READ_PASSIVE  _CLIOC(0x0006)
#define PN532IOC_GET_DATA_READY         _CLIOC(0x0007)
#define PN532IOC_GET_TAG_ID             _CLIOC(0x0008)
#define PN532IOC_GET_STATE              _CLIOC(0x0009)
#define PN532IOC_READ_TAG_DATA          _CLIOC(0x000a)
#define PN532IOC_WRITE_TAG_DATA         _CLIOC(0x000b)

/* Contactless common IOCTL Commands ****************************************/

/* Note about [Placeholder]
 * These are not yet implemented and exist to be changed.
 * But gives an example how and where to implement new features
 */

/* Legacy */

#define CLIOC_READ_MIFARE_DATA          _CLIOC(0x000c)
#define CLIOC_WRITE_MIFARE_DATA         _CLIOC(0x000d)

/* Card mifare type A parameters */

#define CLIOC_TYPE_A_SET_PARAMS         _CLIOC(0x000e) /* arg: ptr to struct ctls_type_a_params_s
                                                        * Configures the card params before scanning */

/* Card mifare type B parameters
 * [Placeholder]
 */

#define CLIOC_TYPE_B_SET_PARAMS         _CLIOC(0x000f) /* arg: ptr to struct ctls_type_b_params_s
                                                        * Configures the card params before scanning */

/* Scan mode */

#define CLIOC_SCAN                      _CLIOC(0x0010) /* Starts scanning with ptr to struct ctls_scan_params_s */

#define CLIOC_AUTO_SCAN                 _CLIOC(0x0011) /* Starts scanning with ptr to struct ctls_auto_params_s */

/* Polling */

#define CLIOC_POLL_STATUS               _CLIOC(0x0012) /* arg: ptr to enum ctls_status_e */

/* Get results */

#define CLIOC_GET_RESULTS               _CLIOC(0x0013) /* Returns cards into ptr to struct ctls_scan_result_s
                                                        * This is a blocking function if scanner is scanning.
                                                        * To avoid blocking, check POLL_STATUS for CTLS_STATUS_RESULT_READY */

/* Constants */

#define CLIOC_MAX_UID_SIZE  16
#define CTLS_TYPE_B_ATTRIB_MAX_LEN 32

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* --- Start of common --- */

/* Common data **************************************************************/

enum ctls_status_e
{
  CTLS_STATUS_OFF,
  CTLS_STATUS_SLEEP,
  CTLS_STATUS_IDLE,
  CTLS_STATUS_SCANNING,
  CTLS_STATUS_RESULT_READY,
  CTLS_STATUS_ERROR
};

struct ctls_uid_s
{
  uint8_t size; /* Typically 4, 7 or 10 */
  uint8_t bytes[CLIOC_MAX_UID_SIZE];
};

enum ctls_type_b_polling_e
{
  CTLS_TYPE_B_TIMESLOT = 0x00,
  CTLS_TYPE_B_PROBABILISTIC = 0x01
};

enum ctls_card_type_e
{
  CTLS_CARD_NONE,     /* This card is not scanned */
  CTLS_CARD_UNKNOWN,  /* This card is unknown */
  CTLS_CARD_ERROR,    /* This card has problems */
  CTLS_CARD_MIFARE_TYPE_A,
  CTLS_CARD_MIFARE_TYPE_B
};

/* Common: Card params ******************************************************/

struct ctls_type_a_params_s
{
  /* Optional field:
   * Enabled only when the host controller
   * wants to initialize a target with a known UID.
   */

  bool enable_target_uid;
  struct ctls_uid_s target_uid;
};

struct ctls_type_b_params_s /* [Placeholder] */
{
  /* Mandatory field:
   * Application Familly Identifier
   */

  uint8_t afi;

  /* Optional field:
   * If disabled, timeslot polling will be used
   */

  bool enable_polling_method;
  enum ctls_type_b_polling_e polling_method;
};

/* Common: Scan params ******************************************************/

struct ctls_scan_params_s
{
  /* Amount of max cards in the same scan */

  uint8_t max_at_once;

  /* Target type */

  enum ctls_card_type_e type;
};

struct ctls_auto_params_s
{
  /* Amount of types to poll */

  size_t number_of_types;

  /* Amount of max cards in the same scan */

  uint8_t max_at_once;

  /* User array of types */

  enum ctls_card_type_e *types;
};

/* Common: Card types *******************************************************/

struct ctls_card_type_a_s
{
  /* The following fields are for multi scan results */

  bool valid; /* Non-valid when this card does not exist or is corrupted */
  uint8_t target_id;

  /* Type A specific */

  uint8_t sens_res[2];
  uint8_t sel_res;
  struct ctls_uid_s uid;

  /* ATS should be implemented here */
};

struct ctls_card_type_b_s
{
  /* The following fields are for multi scan results */

  bool valid; /* Non-valid when this card does not exist or is corrupted */
  uint8_t target_id;

  /* Type B specific */

  uint8_t atqb[12];
  uint8_t attrib_len;
  uint8_t attributes[CTLS_TYPE_B_ATTRIB_MAX_LEN];
};

/* Union card type. Can contain any type */

union ctls_card_u
{
  struct ctls_card_type_a_s type_a;
  struct ctls_card_type_b_s type_b;
};

struct ctls_card_s
{
  enum ctls_card_type_e type;

  /* Contains card info of .type */

  union ctls_card_u info;
};

/* Common: Scan result ******************************************************/

struct ctls_scan_result_s
{
  /* Cards detected in once scan
   * Written by scanner
   */

  uint8_t card_results;

  /* User specified max cards that fit in *cards */

  uint8_t max_cards_results;

  /* User array of (max_cards_results) cards
   * Filled by scanner. Type is included
   */

  struct ctls_card_s *cards;
};

/* --- End of common --- */

/* PICC */

struct picc_uid_s
{
  uint8_t  size;         /* Number of bytes in the UID. 4, 7 or 10 */
  uint8_t  uid_data[10];
  uint8_t  sak;          /* The SAK (Select Acknowledge) return by the PICC */
};

/* Coding of Select Acknowledge (SAK) according to:
 *   http://www.nxp.com/documents/application_note/AN10833.pdf
 */

enum picc_cardid_e
{
  PICC_TYPE_NOT_COMPLETE = 0x04, /* UID not complete */
  PICC_TYPE_ISO_14443_4  = 0x20, /* PICC compliant with ISO/IEC 14443-4 */
  PICC_TYPE_ISO_18092    = 0x40, /* PICC compliant with ISO/IEC 18092 (NFC) */
  PICC_TYPE_MIFARE_MINI  = 0x09,
  PICC_TYPE_MIFARE_1K    = 0x08,
  PICC_TYPE_MIFARE_4K    = 0x18,
  PICC_TYPE_MIFARE_UL    = 0x00,
  PICC_TYPE_MIFARE_PLUS  = 0x11,
  PICC_TYPE_TNP3XXX      = 0x01
};

struct mifare_tag_data_s
{
  uint8_t  data[16];
  uint8_t  address;
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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CONTACTLESS_IOCTL_H */
