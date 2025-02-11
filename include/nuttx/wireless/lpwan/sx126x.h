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
 
 /****************************************************************************
 * Public Data Types
 ****************************************************************************/

struct sx126x_lower_s
{
    unsigned int port;
    void (*reset)(void);
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void sx126x_register(FAR struct spi_dev_s *spi,
                     FAR const struct sx126x_lower_s *lower);

 #endif /* __INCLUDE_NUTTX_WIRELESS_LPWAN_SX126X_H */