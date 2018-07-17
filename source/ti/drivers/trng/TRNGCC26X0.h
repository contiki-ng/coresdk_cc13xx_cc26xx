/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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
 *  @file       TRNGCC26X0.h
 *
 *  @brief      TRNG driver implementation for the CC26X0 family
 *
 * @warning     This is a beta API. It may change in future releases.
 *
 *  This file should only be included in the board file to fill the TRNG_config
 *  struct.

 *  The CC26X0 family has a dedicated hardware TRNG based on sampling multiple
 *  free running oscillators. With all FROs enabled, the TRNG hardware generates
 *  64 bits of entropy approximately every 5ms. The driver implementation
 *  chains multiple 64-bit entropy generation operations together to generate
 *  an arbitrary amount of entropy.
 *
 *  The driver implementation does not perform runtime checks for most input parameters.
 *  Only values that are likely to have a stochastic element to them are checked (such
 *  as whether a driver is already open). Higher input paramter validation coverage is
 *  achieved by turning on assertions when compiling the driver.
 *
 */

#ifndef ti_drivers_TRNG_TRNGCC26X0__include
#define ti_drivers_TRNG_TRNGCC26X0__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/TRNG.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKey.h>

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

/*!
 *  @brief      TRNGCC26X0 Hardware Attributes
 *
 *  TRNG26X0 hardware attributes should be included in the board file
 *  and pointed to by the TRNG_config struct.
 */
typedef struct TRNGCC26X0_HWAttrs {
    /*! @brief Crypto Peripheral's interrupt priority.

        The CC26xx uses three of the priority bits, meaning ~0 has the same effect as (7 << 5).

        (7 << 5) will apply the lowest priority.

        (1 << 5) will apply the highest priority.

        Setting the priority to 0 is not supported by this driver.

        HWI's with priority 0 ignore the HWI dispatcher to support zero-latency interrupts, thus invalidating the critical sections in this driver.
    */
    uint8_t    intPriority;
    /*! @brief TRNG SWI priority.
        The higher the number, the higher the priority.
        The minimum is 0 and the maximum is 15 by default.
        The maximum can be reduced to save RAM by adding or modifying Swi.numPriorities in the kernel configuration file.
    */
    uint32_t   swiPriority;
} TRNGCC26X0_HWAttrs;

/*!
 *  @brief      TRNGCC26XX Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct TRNGCC26X0_Object {
    bool                            isOpen;
    int_fast16_t                    returnStatus;
    TRNG_ReturnBehavior             returnBehavior;
    size_t                          entropyGenerated;
    size_t                          entropyRequested;
    uint32_t                        semaphoreTimeout;
    uint8_t                         *entropyBuffer;
    CryptoKey                       *entropyKey;
    TRNG_CallbackFxn                callbackFxn;
    SwiP_Struct                     callbackSwi;
} TRNGCC26X0_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_TRNG_TRNGCC26X0__include */
