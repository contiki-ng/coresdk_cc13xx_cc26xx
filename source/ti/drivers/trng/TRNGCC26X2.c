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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <ti/drivers/dpl/DebugP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/dpl/DebugP.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26X2.h>
#include <ti/drivers/TRNG.h>
#include <ti/drivers/trng/TRNGCC26X2.h>
#include <ti/drivers/cryptoutils/sharedresources/PKAResourceCC26XX.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKey.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(inc/hw_crypto.h)
#include DeviceFamily_constructPath(inc/hw_pka.h)
#include DeviceFamily_constructPath(inc/hw_pka_ram.h)
#include DeviceFamily_constructPath(driverlib/trng.h)
#include DeviceFamily_constructPath(driverlib/cpu.h)
#include DeviceFamily_constructPath(driverlib/interrupt.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(driverlib/smph.h)
#include DeviceFamily_constructPath(driverlib/pka.h)

/* Macros */
#define MAX(x,y)   (((x) > (y)) ?  (x) : (y))
#define MIN(x,y)   (((x) < (y)) ?  (x) : (y))


#define TRNGCC26X2_SCRATCH_BUFFER ((uint8_t *)(PKA_RAM_BASE + PKA_RAM_TOT_BYTE_SIZE / 2))
#define TRNGCC26X2_SCRATCH_BUFFER_SIZE 768


/* Forward declarations */
static void TRNGCC26X2_swiFxn (uintptr_t arg0, uintptr_t arg1);
static void TRNGCC26X2_internalCallbackFxn (TRNG_Handle handle,
                                            int_fast16_t returnValue,
                                            CryptoKey *entropy);
static int_fast16_t TRNGCC26X2_waitForAccess(TRNG_Handle handle);
static int_fast16_t TRNGCC26X2_waitForResult(TRNG_Handle handle);
static void TRNGCC26X2_constructRTOSObjects(void);
static void TRNGCC26X2_destructRTOSObjects(void);
static void TRNGCC26X2_copyEntropy(uint32_t interruptStatus, TRNGCC26X2_Object *object);
static void TRNG_restartFRO(uint32_t interruptStatus);
static void TRNGCC26X2_basicHwiFxn (uintptr_t arg0);
static void TRNGCC26X2_entropyLessThanHwiFxn (uintptr_t arg0);
static void TRNGCC26X2_pkaNonZeroHwiFxn (uintptr_t arg0);
static void TRNGCC26X2_pkaHwiFxn (uintptr_t arg0);

/* Extern globals */
extern const TRNG_Config TRNG_config[];
extern const uint_least8_t TRNG_count;

/* Static globals */
static bool isInitialized = false;

/* TRNG driver semaphore used to synchronize accesses to the TRNG module */
static SemaphoreP_Struct TRNGCC26X2_accessSemaphore;
static SemaphoreP_Struct TRNGCC26X2_operationSemaphore;

volatile bool TRNGCC26X2_pollingFlag = 0;

static HwiP_Struct TRNGCC26X2_hwi;

static uint8_t openCount = 0;

static uint32_t resultAddress;

static void errorSpin(uintptr_t arg) {
    while(1);
}

/*
 *  ======== TRNGCC26X2_constructRTOSObjects ========
 */
static void TRNGCC26X2_constructRTOSObjects(void) {
    HwiP_Params hwiParams;

    if (openCount == 0){
        /* Construct the common Hwi with a dummy ISR function. This should not matter as the function is set
         * whenever we start an operation after pending on TRNGCC26X2_accessSemaphore
         */
        HwiP_Params_init(&hwiParams);
        HwiP_construct(&(TRNGCC26X2_hwi), INT_TRNG_IRQ, errorSpin, &hwiParams);

        SemaphoreP_constructBinary(&TRNGCC26X2_accessSemaphore, 1);
        SemaphoreP_constructBinary(&TRNGCC26X2_operationSemaphore, 0);

    }

    openCount++;
}

/*
 *  ======== TRNGCC26X2_destructRTOSObjects ========
 */
static void TRNGCC26X2_destructRTOSObjects(void) {
    if (openCount == 1){
        HwiP_destruct(&TRNGCC26X2_hwi);
        SemaphoreP_destruct(&TRNGCC26X2_operationSemaphore);
        SemaphoreP_destruct(&TRNGCC26X2_accessSemaphore);
    }

    openCount--;
}

/*
 *  ======== TRNGCC26X2_swiFxn ========
 */
static void TRNGCC26X2_swiFxn (uintptr_t arg0, uintptr_t arg1) {
    TRNGCC26X2_Object *object = ((TRNG_Handle)arg0)->object;

    /*  Grant access for other threads to use the crypto module.
     *  The semaphore must be posted before the callbackFxn to allow the chaining
     *  of operations.
     */
    SemaphoreP_post(&TRNGCC26X2_accessSemaphore);

    Power_releaseConstraint(PowerCC26XX_DISALLOW_STANDBY);

    object->callbackFxn((TRNG_Handle)arg0,
                        object->returnStatus,
                        object->entropyKey);
}

static void TRNGCC26X2_copyEntropy(uint32_t interruptStatus, TRNGCC26X2_Object *object) {
    uint8_t tmpEntropyBuf[2 * sizeof(uint32_t)];
    size_t bytesToCopy = 0;

    if (interruptStatus & TRNG_IRQFLAGSTAT_RDY_M) {
        ((uint32_t *)tmpEntropyBuf)[0] = TRNGNumberGet(TRNG_LOW_WORD);
        ((uint32_t *)tmpEntropyBuf)[1] = TRNGNumberGet(TRNG_HI_WORD);

        bytesToCopy =  MIN(object->entropyRequested - object->entropyGenerated, 2 * sizeof(uint32_t));

        memcpy(object->entropyBuffer + object->entropyGenerated,
               tmpEntropyBuf,
               bytesToCopy);

        object->entropyGenerated += bytesToCopy;
    }
}

static void TRNG_restartFRO(uint32_t interruptStatus) {
     if (interruptStatus & TRNG_IRQFLAGSTAT_SHUTDOWN_OVF_M) {
        uint32_t froAlarmMask;

        froAlarmMask = HWREG(TRNG_BASE + TRNG_O_ALARMSTOP);

        // Clear alarms for FROs that exhibited repeating pattern
        HWREG(TRNG_BASE + TRNG_O_ALARMMASK) = 0;

        // Clear alarms for FROs that stopped
        HWREG(TRNG_BASE + TRNG_O_ALARMSTOP) = 0;

        // De-tune the FROs that had an alarm to attempt to
        // break their lock-in on SCLK_HF
        HWREG(TRNG_BASE + TRNG_O_FRODETUNE) = froAlarmMask;

        // Re-enable the FROs
        HWREG(TRNG_BASE + TRNG_O_FROEN) |= froAlarmMask;
    }
}

/*
 *  ======== TRNGCC26X2_basicHwiFxn ========
 */
static void TRNGCC26X2_basicHwiFxn (uintptr_t arg0) {
    TRNGCC26X2_Object *object = ((TRNG_Handle)arg0)->object;
    uint32_t interruptStatus;

    interruptStatus = HWREG(TRNG_BASE + TRNG_O_IRQFLAGSTAT);
    TRNGIntClear(TRNG_NUMBER_READY | TRNG_FRO_SHUTDOWN);

    TRNGCC26X2_copyEntropy(interruptStatus, object);

    TRNG_restartFRO(interruptStatus);

    if (object->entropyGenerated >= object->entropyRequested) {

        TRNGDisable();

        object->returnStatus = TRNG_STATUS_SUCCESS;

        SwiP_post(&(object->callbackSwi));
    }

}

/*
 *  ======== TRNGCC26X2_entropyLessThanHwiFxn ========
 */
static void TRNGCC26X2_entropyLessThanHwiFxn (uintptr_t arg0) {
    TRNGCC26X2_Object *object = ((TRNG_Handle)arg0)->object;
    uint32_t interruptStatus;

    interruptStatus = HWREG(TRNG_BASE + TRNG_O_IRQFLAGSTAT);
    TRNGIntClear(TRNG_NUMBER_READY | TRNG_FRO_SHUTDOWN);

    TRNGCC26X2_copyEntropy(interruptStatus, object);

    TRNG_restartFRO(interruptStatus);

    if (object->entropyGenerated >= object->entropyRequested) {

        TRNGDisable();

        PKABigNumModStart(object->entropyBuffer,
                          object->entropyRequested,
                          object->upperBound,
                          object->entropyKey->u.plaintext.keyLength,
                          &resultAddress);

        IntPendClear(INT_PKA_IRQ);
        IntEnable(INT_PKA_IRQ);
    }
}

/*
 *  ======== TRNGCC26X2_pkaHwiFxn ========
 */
static void TRNGCC26X2_pkaHwiFxn (uintptr_t arg0) {
    TRNGCC26X2_Object *object = ((TRNG_Handle)arg0)->object;
    uint32_t pkaResult;

    // Disable interrupt again
    IntDisable(INT_PKA_IRQ);

    pkaResult = PKABigNumModGetResult(object->entropyKey->u.plaintext.keyMaterial,
                                      object->entropyKey->u.plaintext.keyLength,
                                      resultAddress);

    object->returnStatus = pkaResult == PKA_STATUS_SUCCESS ? TRNG_STATUS_SUCCESS : TRNG_STATUS_ERROR;

    SwiP_post(&(object->callbackSwi));
}

/*
 *  ======== TRNGCC26X2_pkaNonZeroHwiFxn ========
 */
static void TRNGCC26X2_pkaNonZeroHwiFxn (uintptr_t arg0) {
    TRNGCC26X2_Object *object = ((TRNG_Handle)arg0)->object;
    uint32_t pkaResult;

    // Disable interrupt again
    IntDisable(INT_PKA_IRQ);

    pkaResult = PKABigNumModGetResult(object->entropyKey->u.plaintext.keyMaterial,
                                      object->entropyKey->u.plaintext.keyLength,
                                      resultAddress);

    /* Unlike BSI Technical Guideline TR-03111, we do not do modulo upperBound - 1
     * and then add one to the result to ensure we get a non-zero number.
     * We practice rejection sampling instead.
     * If we happen to get all zeros or a multiple of upperBound, we generate
     * a new number instead. This is an incredibly improbable event so
     * rejection sampling is highly efficient while reducing complexity.
     */
    if (PKAArrayAllZeros(object->entropyKey->u.plaintext.keyMaterial,
                         object->entropyKey->u.plaintext.keyLength)) {
        /* Restart operation and generate more entropy if we happened
         * to generate an all-zeros or upperBound-multiple vector
         */
        object->entropyGenerated = 0;

        /* Set startup cycles and reload cycles to 240000.
         * This is equivalent to 5ms at 48MHz which generates
         * 64 bits of entropy with all 24 FROs enabled.
         */
        TRNGConfigure(0, 240000, 0);
        TRNGEnable();
    }
    else {
        /* Clean up and run callback */
        object->returnStatus = pkaResult == PKA_STATUS_SUCCESS ? TRNG_STATUS_SUCCESS : TRNG_STATUS_ERROR;

        SwiP_post(&(object->callbackSwi));
    }
}


/*
 *  ======== TRNGCC26X2_internalCallbackFxn ========
 */
static void TRNGCC26X2_internalCallbackFxn (TRNG_Handle handle,
                                            int_fast16_t returnValue,
                                            CryptoKey *entropy) {
    TRNGCC26X2_Object *object = handle->object;

    /* This function is only ever registered when in TRNG_RETURN_BEHAVIOR_BLOCKING
     * or TRNG_RETURN_BEHAVIOR_POLLING.
     */
    if (object->returnBehavior == TRNG_RETURN_BEHAVIOR_BLOCKING) {
        SemaphoreP_post(&TRNGCC26X2_operationSemaphore);
    }
    else {
        TRNGCC26X2_pollingFlag = 1;
    }
}

/*
 *  ======== TRNG_init ========
 */
void TRNG_init(void) {
    uint_least8_t i;
    uint_fast8_t key;

    key = HwiP_disable();

    if (!isInitialized) {
        /* Call each instances' driver init function */
        for (i = 0; i < TRNG_count; i++) {
            TRNG_Handle handle = (TRNG_Handle)&(TRNG_config[i]);
            TRNGCC26X2_Object *object = (TRNGCC26X2_Object *)handle->object;
            object->isOpen = false;
        }

        isInitialized = true;
    }

    HwiP_restore(key);
}

/*
 *  ======== TRNG_open ========
 */
TRNG_Handle TRNG_open(uint_least8_t index, TRNG_Params *params) {
    SwiP_Params                 swiParams;
    TRNG_Handle                 handle;
    TRNGCC26X2_Object           *object;
    TRNGCC26X2_HWAttrs const    *hwAttrs;
    uint_fast8_t                key;

    handle = (TRNG_Handle)&(TRNG_config[index]);
    object = handle->object;
    hwAttrs = handle->hwAttrs;


    key = HwiP_disable();

    if (!isInitialized || object->isOpen) {
        HwiP_restore(key);
        return NULL;
    }

    object->isOpen = true;

    /* If params are NULL, use defaults */
    if (params == NULL) {
        params = (TRNG_Params *)&TRNG_defaultParams;
    }

    object->returnBehavior = params->returnBehavior;
    object->callbackFxn = params->returnBehavior == TRNG_RETURN_BEHAVIOR_CALLBACK ? params->callbackFxn : TRNGCC26X2_internalCallbackFxn;
    object->semaphoreTimeout = params->timeout;

    /* Create Swi object for this TRNG peripheral */
    SwiP_Params_init(&swiParams);
    swiParams.arg0 = (uintptr_t)handle;
    swiParams.priority = hwAttrs->swiPriority;
    SwiP_construct(&(object->callbackSwi), TRNGCC26X2_swiFxn, &swiParams);

    TRNGCC26X2_constructRTOSObjects();

    PKAResourceCC26XX_constructRTOSObjects();

    /* Set power dependency - i.e. power up and enable clock for TRNG (TRNGCC26X2) module. */
    Power_setDependency(PowerCC26XX_PERIPH_TRNG);
    Power_setDependency(PowerCC26X2_PERIPH_PKA);

    HwiP_restore(key);

    return handle;
}

/*
 *  ======== TRNG_close ========
 */
void TRNG_close(TRNG_Handle handle) {
    TRNGCC26X2_Object         *object;

    uint_fast8_t key;

    key = HwiP_disable();

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;

    TRNGCC26X2_destructRTOSObjects();

    PKAResourceCC26XX_destructRTOSObjects();

    /* Destroy the Swi */
    SwiP_destruct(&(object->callbackSwi));

    /* Release power dependency on TRNG Module. */
    Power_releaseDependency(PowerCC26XX_PERIPH_TRNG);
    Power_releaseDependency(PowerCC26X2_PERIPH_PKA);

    /* Mark the module as available */
    object->isOpen = false;

    HwiP_restore(key);
}

/*
 *  ======== TRNGCC26X2_waitForAccess ========
 */
static int_fast16_t TRNGCC26X2_waitForAccess(TRNG_Handle handle) {
    TRNGCC26X2_Object *object = handle->object;
    uint32_t timeout;

    /* Set to SemaphoreP_NO_WAIT to start operations from SWI or HWI context */
    timeout = object->returnBehavior == TRNG_RETURN_BEHAVIOR_BLOCKING ? object->semaphoreTimeout : SemaphoreP_NO_WAIT;

    return SemaphoreP_pend(&TRNGCC26X2_accessSemaphore, timeout);
}

/*
 *  ======== TRNGCC26X2_waitForAccess ========
 */
static int_fast16_t TRNGCC26X2_waitForPKAAccess(TRNG_Handle handle) {
    TRNGCC26X2_Object *object = handle->object;
    uint32_t timeout;

    /* Set to SemaphoreP_NO_WAIT to start operations from SWI or HWI context */
    timeout = object->returnBehavior == TRNG_RETURN_BEHAVIOR_BLOCKING ? object->semaphoreTimeout : SemaphoreP_NO_WAIT;

    return SemaphoreP_pend(&PKAResourceCC26XX_accessSemaphore, timeout);
}

/*
 *  ======== TRNGCC26X2_waitForResult ========
 */
static int_fast16_t TRNGCC26X2_waitForResult(TRNG_Handle handle){
    TRNGCC26X2_Object *object = handle->object;

    if (object->returnBehavior == TRNG_RETURN_BEHAVIOR_POLLING) {
        while(!TRNGCC26X2_pollingFlag);
        return object->returnStatus;
    }
    else if (object->returnBehavior == TRNG_RETURN_BEHAVIOR_BLOCKING) {
        SemaphoreP_pend(&TRNGCC26X2_operationSemaphore, SemaphoreP_WAIT_FOREVER);
        return object->returnStatus;
    }
    else {
        return TRNG_STATUS_SUCCESS;
    }
}


/*
 *  ======== TRNG_generateEntropy ========
 */
int_fast16_t TRNG_generateEntropy(TRNG_Handle handle, CryptoKey *entropy) {
    TRNGCC26X2_Object *object = handle->object;
    TRNGCC26X2_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Try and obtain access to the TRNG module */
    if (TRNGCC26X2_waitForAccess(handle) != SemaphoreP_OK) {
        return TRNG_STATUS_RESOURCE_UNAVAILABLE;
    }

    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);

    object->entropyGenerated = 0;
    object->entropyKey = entropy;
    object->entropyBuffer = entropy->u.plaintext.keyMaterial;
    object->entropyRequested = entropy->u.plaintext.keyLength;

    /* We need to set the HWI function and priority since the same physical interrupt is shared by multiple
     * drivers and they all need to coexist. Whenever a driver starts an operation, it
     * registers its HWI callback with the OS.
     */
    HwiP_setFunc(&TRNGCC26X2_hwi, TRNGCC26X2_basicHwiFxn, (uintptr_t)handle);
    HwiP_setPriority(INT_TRNG_IRQ, hwAttrs->intPriority);


    TRNGCC26X2_pollingFlag = 0;
    TRNGIntEnable(TRNG_NUMBER_READY | TRNG_FRO_SHUTDOWN);

    /* Set startup cycles and reload cycles to 240000.
     * This is equivalent to 5ms at 48MHz which generates
     * 64 bits of entropy with all 24 FROs enabled.
     */
    TRNGConfigure(0, 240000, 0);
    TRNGEnable();

    return TRNGCC26X2_waitForResult(handle);
}

/*
 *  ======== TRNG_generateEntropyLessThan ========
 */
int_fast16_t TRNG_generateEntropyLessThan(TRNG_Handle handle, CryptoKey *entropy, const uint8_t *upperBound) {
    TRNGCC26X2_Object *object = handle->object;
    TRNGCC26X2_HWAttrs const *hwAttrs = handle->hwAttrs;

    if (TRNGCC26X2_waitForPKAAccess(handle) != SemaphoreP_OK){
        return TRNG_STATUS_RESOURCE_UNAVAILABLE;
    }

    /* Try and obtain access to the TRNG module */
    if (TRNGCC26X2_waitForAccess(handle) != SemaphoreP_OK) {
        /* Release the PKA access semaphore that we acquired above. */
        SemaphoreP_post(&PKAResourceCC26XX_accessSemaphore);

        return TRNG_STATUS_RESOURCE_UNAVAILABLE;
    }

    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);

    object->entropyGenerated = 0;
    object->entropyKey = entropy;
    object->upperBound = upperBound;

    /* Copy initial entropy into sctratch buffer and then take that modulo
     * upperBound before copying it back into the entropy CryptoKey
     */
    object->entropyBuffer = TRNGCC26X2_SCRATCH_BUFFER;

    /* Request 64 bits of extra entropy in order to ensure approximately uniform distribution
     * of entropy after taking result modulo upperBound.
     * This corresponds to the BSI recommendation in BSI Technical Guideline TR-03111.
     * Generating an extra 64 bits and taking the result modulo upperBound leaves you with a
     * negligible bias.
     */
    object->entropyRequested = entropy->u.plaintext.keyLength + 8;

    /* We need to set the HWI function and priority since the same physical interrupt is shared by multiple
     * drivers and they all need to coexist. Whenever a driver starts an operation, it
     * registers its HWI callback with the OS.
     */
    HwiP_setFunc(&TRNGCC26X2_hwi, TRNGCC26X2_entropyLessThanHwiFxn, (uintptr_t)handle);
    HwiP_setPriority(INT_TRNG_IRQ, hwAttrs->intPriority);

    HwiP_setFunc(&PKAResourceCC26XX_hwi, TRNGCC26X2_pkaHwiFxn, (uintptr_t)handle);
    HwiP_setPriority(INT_PKA_IRQ, hwAttrs->intPriority);


    TRNGCC26X2_pollingFlag = 0;
    TRNGIntEnable(TRNG_NUMBER_READY | TRNG_FRO_SHUTDOWN);

    /* Set startup cycles and reload cycles to 240000.
     * This is equivalent to 5ms at 48MHz which generates
     * 64 bits of entropy with all 24 FROs enabled.
     */
    TRNGConfigure(0, 240000, 0);
    TRNGEnable();

    return TRNGCC26X2_waitForResult(handle);
}

int_fast16_t TRNG_generateEntropyNonZeroLessThan(TRNG_Handle handle, CryptoKey *entropy, const uint8_t *upperBound) {
        TRNGCC26X2_Object *object = handle->object;
    TRNGCC26X2_HWAttrs const *hwAttrs = handle->hwAttrs;

    if (TRNGCC26X2_waitForPKAAccess(handle) != SemaphoreP_OK){
        return TRNG_STATUS_RESOURCE_UNAVAILABLE;
    }

    /* Try and obtain access to the TRNG module */
    if (TRNGCC26X2_waitForAccess(handle) != SemaphoreP_OK) {
        /* Release the PKA access semaphore that we acquired above. */
        SemaphoreP_post(&PKAResourceCC26XX_accessSemaphore);

        return TRNG_STATUS_RESOURCE_UNAVAILABLE;
    }

    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);

    object->entropyGenerated = 0;
    object->entropyKey = entropy;
    object->upperBound = upperBound;

    /* Copy initial entropy into sctratch buffer and then take that modulo
     * upperBound before copying it back into the entropy CryptoKey
     */
    object->entropyBuffer = TRNGCC26X2_SCRATCH_BUFFER;

    /* Request 64 bits of extra entropy in order to ensure approximately uniform distribution
     * of entropy after taking result modulo upperBound.
     * This corresponds to the BSI recommendation in BSI Technical Guideline TR-03111.
     * Generating an extra 64 bits and taking the result modulo upperBound leaves you with a
     * negligible bias.
     */
    object->entropyRequested = entropy->u.plaintext.keyLength + 8;

    /* We need to set the HWI function and priority since the same physical interrupt is shared by multiple
     * drivers and they all need to coexist. Whenever a driver starts an operation, it
     * registers its HWI callback with the OS.
     */
    HwiP_setFunc(&TRNGCC26X2_hwi, TRNGCC26X2_entropyLessThanHwiFxn, (uintptr_t)handle);
    HwiP_setPriority(INT_TRNG_IRQ, hwAttrs->intPriority);

    HwiP_setFunc(&PKAResourceCC26XX_hwi, TRNGCC26X2_pkaNonZeroHwiFxn, (uintptr_t)handle);
    HwiP_setPriority(INT_PKA_IRQ, hwAttrs->intPriority);


    TRNGCC26X2_pollingFlag = 0;
    TRNGIntEnable(TRNG_NUMBER_READY | TRNG_FRO_SHUTDOWN);

    /* Set startup cycles and reload cycles to 240000.
     * This is equivalent to 5ms at 48MHz which generates
     * 64 bits of entropy with all 24 FROs enabled.
     */
    TRNGConfigure(0, 240000, 0);
    TRNGEnable();

    return TRNGCC26X2_waitForResult(handle);
}
