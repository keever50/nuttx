/****************************************************************************
 * include/nuttx/wireless/lpwan/sx127x.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_LPWAN_SX126X_H
#define __INCLUDE_NUTTX_WIRELESS_LPWAN_SX126X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/ioctl.h>

#include <stdint.h>
#include <stdbool.h>
 
 /***************************************************************************
  * Defintions
  ***************************************************************************/

#define SX126X_RX_PAYLOAD_SIZE              0xff
  
/* IRQ Register bits ********************************************************/

#define SX126X_IRQ_TXDONE_MASK              (1<<0)
#define SX126X_IRQ_RXDONE_MASK              (1<<1)
#define SX126X_IRQ_PREAMBLEDETECTED_MASK    (1<<2)
#define SX126X_IRQ_SYNCWORDVALID_MASK       (1<<3)
#define SX126X_IRQ_HEADERVALID_MASK         (1<<4)
#define SX126X_IRQ_HEADERERR_MASK           (1<<5)
#define SX126X_IRQ_CRCERR_MASK              (1<<6)
#define SX126X_IRQ_CADDONE_MASK             (1<<7)
#define SX126X_IRQ_CADDETECTED_MASK         (1<<8)
#define SX126X_IRQ_TIMEOUT_MASK             (1<<9)
#define SX126X_IRQ_LRFHSSHOP_MASK           (1<<14)

/* Others */

#define SX126X_NOP                   0
#define SX126X_NO_TIMEOUT            0
#define SX126X_NO_DELAY              0

/* Oscillators and PLLs */

#define SX126X_OSC_MAIN_HZ           (32000000)
#define SX126X_FXTAL                 SX126X_OSC_MAIN_HZ
#define SX126X_PLL_STEP_SHIFT_AMOUNT (14)
#define SX126X_PLL_STEP_SCALED       (SX126X_FXTAL>>(25-SX126X_PLL_STEP_SHIFT_AMOUNT))

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

/* Standby config */

enum sx126x_standby_mode_e
{
  SX126X_STDBY_RC    = 0x00,
  SX126X_STDBY_XOSC  = 0x01
};

/* Packet Types */

enum sx126x_packet_type_e
{
  SX126X_PACKETTYPE_GFSK    = 0x00,
  SX126X_PACKETTYPE_LORA    = 0x01,
  SX126X_PACKETTYPE_LR_FHSS = 0x13
};

/* Ramp times */

enum sx126x_ramp_time_e
{
  SX126X_SET_RAMP_10U   = 0x00,
  SX126X_SET_RAMP_20U   = 0x01,
  SX126X_SET_RAMP_40U   = 0x02,
  SX126X_SET_RAMP_80U   = 0x03,
  SX126X_SET_RAMP_200U  = 0x04,
  SX126X_SET_RAMP_800U  = 0x05,
  SX126X_SET_RAMP_1700U = 0x06,
  SX126X_SET_RAMP_3400U = 0x07
};

/* GFSK Pulse shapes */

enum sx126x_gfsk_pulseshape_e
{
  SX126X_GFSK_PULSESHAPE_NONE            = 0x00,
  SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_0_3 = 0x08,
  SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_0_5 = 0x09,
  SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_0_7 = 0x0a,
  SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_1   = 0x0b
};

/* GFSK Bandwidths in Hz */

enum sx126x_gfsk_bandwidth_e
{
  SX126X_GFSK_BANDWIDTH_4800HZ   = 0x1f,
  SX126X_GFSK_BANDWIDTH_5800HZ   = 0x17,
  SX126X_GFSK_BANDWIDTH_7300HZ   = 0x0f,
  SX126X_GFSK_BANDWIDTH_9700HZ   = 0x1e,
  SX126X_GFSK_BANDWIDTH_11700HZ  = 0x16,
  SX126X_GFSK_BANDWIDTH_14600HZ  = 0x0e,
  SX126X_GFSK_BANDWIDTH_19500HZ  = 0x1d,
  SX126X_GFSK_BANDWIDTH_23400HZ  = 0x15,
  SX126X_GFSK_BANDWIDTH_29300HZ  = 0x0d,
  SX126X_GFSK_BANDWIDTH_39000HZ  = 0x1c,
  SX126X_GFSK_BANDWIDTH_46900HZ  = 0x14,
  SX126X_GFSK_BANDWIDTH_58600HZ  = 0x0c,
  SX126X_GFSK_BANDWIDTH_78200HZ  = 0x1b,
  SX126X_GFSK_BANDWIDTH_93800HZ  = 0x13,
  SX126X_GFSK_BANDWIDTH_117300HZ = 0x0b,
  SX126X_GFSK_BANDWIDTH_156200HZ = 0x1a,
  SX126X_GFSK_BANDWIDTH_187200HZ = 0x12,
  SX126X_GFSK_BANDWIDTH_234300HZ = 0x0a,
  SX126X_GFSK_BANDWIDTH_312000HZ = 0x19,
  SX126X_GFSK_BANDWIDTH_373600HZ = 0x11,
  SX126X_GFSK_BANDWIDTH_467000HZ = 0x09
};

/* LoRa Spreading Factors */

enum sx126x_lora_sf_e
{
  SX126X_LORA_SF5  = 0x05,
  SX126X_LORA_SF6  = 0x06,
  SX126X_LORA_SF7  = 0x07,
  SX126X_LORA_SF8  = 0x08,
  SX126X_LORA_SF9  = 0x09,
  SX126X_LORA_SF10 = 0x0a,
  SX126X_LORA_SF11 = 0x0b,
  SX126X_LORA_SF12 = 0x0c
};

/* LoRa Bandwidths */

enum sx126x_lora_bw_e
{
  SX126X_LORA_BW_7   = 0x00,
  SX126X_LORA_BW_10  = 0x08,
  SX126X_LORA_BW_15  = 0x01,
  SX126X_LORA_BW_20  = 0x09,
  SX126X_LORA_BW_31  = 0x02,
  SX126X_LORA_BW_41  = 0x0a,
  SX126X_LORA_BW_62  = 0x03,
  SX126X_LORA_BW_125 = 0x04,
  SX126X_LORA_BW_250 = 0x05,
  SX126X_LORA_BW_500 = 0x06
};

/* LoRa Coding Rates */

enum sx126x_lora_cr_e
{
  SX126X_LORA_CR_4_5 = 0x01,
  SX126X_LORA_CR_4_6 = 0x02,
  SX126X_LORA_CR_4_7 = 0x03,
  SX126X_LORA_CR_4_8 = 0x04
};

/* CAD Exit modes */

enum sx126x_cad_exit_mode_e
{
  SX126X_CAD_ONLY = 0x00,
  SX126X_CAD_RX   = 0x01
};

/* TCXO voltages */

enum sx126x_tcxo_voltage_e
{
  SX126X_TCXO_1_6V = 0x00,
  SX126X_TCXO_1_7V = 0x01,
  SX126X_TCXO_1_8V = 0x02,
  SX126X_TCXO_2_2V = 0x03,
  SX126X_TCXO_2_4V = 0x04,
  SX126X_TCXO_2_7V = 0x05,
  SX126X_TCXO_3_0V = 0x06,
  SX126X_TCXO_3_3V = 0x07
};

/* Fallback modes */

enum sx126x_fallback_mode_e
{
  SX126X_FALLBACK_FS          = 0x40,
  SX126X_FALLBACK_STDBY_XOSC  = 0x30,
  SX126X_FALLBACK_STDBY_RC    = 0x20
};

/* Regulator modes */

enum sx126x_regulator_mode_e
{
  SX126X_LDO         = 0x00,
  SX126X_DC_DC_LDO   = 0x01
};

/* Device */

enum sx126x_device_e
{
  SX1261 =            0x01,
  SX1262 =            0x00
};

enum sx126x_gfsk_preamble_detect_e
{
  SX126X_GFSK_PREAMBLE_DETECT_OFF,
  SX126X_GFSK_PREAMBLE_DETECT_8B,
  SX126X_GFSK_PREAMBLE_DETECT_16B,
  SX126X_GFSK_PREAMBLE_DETECT_24B,
  SX126X_GFSK_PREAMBLE_DETECT_32B
};

/* Addr comp */

enum sx126x_address_filtering_e
{
  SX126X_ADDR_FILT_DISABLED,
  SX126X_ADDR_FILT_NODE,
  SX126X_ADDR_FILT_NODE_BROADCAST
};

/* GFSK CRC types */

enum sx126x_gfsk_crc_type_e
{
  SX126X_GFSK_CRCTYPE_OFF         = 0x01,
  SX126X_GFSK_CRCTYPE_1_BYTE      = 0x00,
  SX126X_GFSK_CRCTYPE_2_BYTE      = 0x02,
  SX126X_GFSK_CRCTYPE_1_BYTE_INV  = 0x04,
  SX126X_GFSK_CRCTYPE_2_BYTE_INV  = 0x06
};

/* LoRa mod params */

struct sx126x_modparams_lora_s
{
  enum sx126x_lora_sf_e spreading_factor;
  enum sx126x_lora_bw_e bandwidth;
  enum sx126x_lora_cr_e coding_rate;
  bool low_datarate_optimization;
};

/* GFSK mod params */

struct sx126x_modparams_gfsk_s
{
  uint32_t bitrate;
  enum sx126x_gfsk_pulseshape_e pulseshape;
  enum sx126x_gfsk_bandwidth_e bandwidth;
  uint32_t frequency_deviation;
};

/* LoRa packet params */

struct sx126x_packetparams_lora_s
{
  uint16_t preambles;
  bool fixed_length_header;
  uint8_t payload_length;
  bool crc_enable;
  bool invert_iq;
};

/* GFSK packet params */

struct sx126x_packetparams_gfsk_s
{
  uint16_t preambles;
  enum sx126x_gfsk_preamble_detect_e preamble_detect;
  uint8_t syncword_length;
  enum sx126x_address_filtering_e address_filtering;
  bool include_packet_size;
  uint8_t packet_length;
  enum sx126x_gfsk_crc_type_e crc_type;
  bool whitening_enable;
};

/* Lower driver *************************************************************/

struct sx126x_irq_masks
{
  uint16_t dio1_mask;
  uint16_t dio2_mask;
  uint16_t dio3_mask;
};

struct sx126x_lower_s
{
  /* Index of radio to register.
    * ex: 0 is the primary radio, 1 is the secondary.
    * Must be within the maximum configured radios.
    */

  unsigned int dev_number;
  CODE void (*reset)(void);
  struct sx126x_irq_masks masks;

  /* Interrupt attachments. These should be connected to one of the DIOx pins */

  CODE int (*irq0attach)(xcpt_t handler, FAR void *arg);
};

/* Upper ********************************************************************/

struct sx126x_read_header_s
{
  uint8_t payload_length;
  float snr;
  int16_t rssi_db;
  uint8_t payload[SX126X_RX_PAYLOAD_SIZE];
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void sx126x_register(FAR struct spi_dev_s *spi,
                     FAR const struct sx126x_lower_s *lower,
                     const char* path);

#endif /* __INCLUDE_NUTTX_WIRELESS_LPWAN_SX126X_H */