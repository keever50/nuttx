/****************************************************************************
 * arch/arm/src/samv7/sam_wdt.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_WDT_H
#define __ARCH_ARM_SRC_SAMV7_SAM_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_wdt.h"

#ifdef CONFIG_WATCHDOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_wdt_initialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the WDT hardware.
 *   This interface should be provided by all configurations using
 *   to avoid exposed platform-dependent logic.
 *
 *   At a minimum, this function should call watchdog_register().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sam_wdt_initialize(void);

/****************************************************************************
 * Name: sam_rswdt_initialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the RSWDT hardware.
 *   This interface should be provided by all configurations using
 *   to avoid exposed platform-dependent logic.
 *
 *   At a minimum, this function should call watchdog_register().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sam_rswdt_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_WATCHDOG */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_WDT_H */
