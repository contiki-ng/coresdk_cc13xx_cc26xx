/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       SHA2CC26X2.h
 *
 *  @brief      SHA2 driver implementation for the CC26X2 family
 *
 * @warning     This is a beta API. It may change in future releases.
 *
 *  This file should only be included in the board file to fill the SHA2_config
 *  struct.
 *
 *  The CC26XX family has a dedicated hardware crypto accelerator. It is capable
 *  of multiple AES block cipher modes of operation as well as SHA2 operations.
 *  Only one operation can be carried out on the accerator at a time. Mutual
 *  exclusion is implemented at the driver level and coordinated between all
 *  drivers relying onthe accelerator. It is transparent to the application
 *  and only noted ensure sensible access timeouts are set.
 *
 *  The driver implementation does not perform runtime checks for most input parameters.
 *  Only values that are likely to have a stochastic element to them are checked (such
 *  as whether a driver is already open). Higher input paramter validation coverage is
 *  achieved by turning on assertions when compiling the driver.
 *
 */

#ifndef ti_drivers_sha2_SHA2CC26XX__include
#define ti_drivers_sha2_SHA2CC26XX__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/SHA2.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKey.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/crypto.h)

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

/*!
 *  @brief      SHA2CC26XX Hardware Attributes
 *
 *  SHA226XX hardware attributes should be included in the board file
 *  and pointed to by the SHA2_config struct.
 */
typedef struct SHA2CC26X2_HWAttrs_ {
    /*! @brief Crypto Peripheral's interrupt priority.

        The CC26xx uses three of the priority bits, meaning ~0 has the same effect as (7 << 5).

        (7 << 5) will apply the lowest priority.

        (1 << 5) will apply the highest priority.

        Setting the priority to 0 is not supported by this driver.

        HWI's with priority 0 ignore the HWI dispatcher to support zero-latency interrupts, thus invalidating the critical sections in this driver.
    */
    uint8_t    intPriority;
    /*! @brief SHA2 SWI priority.
        The higher the number, the higher the priority.
        The minimum is 0 and the maximum is 15 by default.
        The maximum can be reduced to save RAM by adding or modifying Swi.numPriorities in the kernel configuration file.
    */
    uint32_t   swiPriority;
} SHA2CC26X2_HWAttrs;

/*!
 *  @brief      SHA2CC26XX Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct SHA2CC26X2_Object_ {
    bool                            isOpen;
    int_fast16_t                    returnStatus;
    SHA2_ReturnBehavior             returnBehavior;
    uint32_t                        semaphoreTimeout;
    SHA2_CallbackFxn                callbackFxn;
    SHA2_Operation                  operation;
    SHA2_OperationType              operationType;
    SwiP_Struct                     callbackSwi;
} SHA2CC26X2_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_sha2_SHA2CC26XX__include */
