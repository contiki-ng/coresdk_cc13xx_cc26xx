/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
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
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/SHA2.h>
#include <ti/drivers/sha2/SHA2CC26X2.h>
#include <ti/drivers/cryptoutils/sharedresources/CryptoResourceCC26XX.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKey.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(inc/hw_crypto.h)
#include DeviceFamily_constructPath(driverlib/sha2.h)
#include DeviceFamily_constructPath(driverlib/cpu.h)
#include DeviceFamily_constructPath(driverlib/interrupt.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(driverlib/smph.h)


/* Forward declarations */
static void SHA2_hwiFxn (uintptr_t arg0);
static void SHA2_swiFxn (uintptr_t arg0, uintptr_t arg1);
static int_fast16_t SHA2_waitForAccess(SHA2_Handle handle);
static int_fast16_t SHA2_waitForResult(SHA2_Handle handle);

/* Extern globals */
extern const SHA2_Config SHA2_config[];
extern const uint_least8_t SHA2_count;

/* Static globals */
static bool isInitialized = false;
static const uint32_t hashSizeTable[] = {
    SHA2_MODE_SELECT_SHA224,
    SHA2_MODE_SELECT_SHA256,
    SHA2_MODE_SELECT_SHA384,
    SHA2_MODE_SELECT_SHA512
};

/*
 *  ======== SHA2_swiFxn ========
 */
static void SHA2_swiFxn (uintptr_t arg0, uintptr_t arg1) {
    SHA2CC26X2_Object *object = ((SHA2_Handle)arg0)->object;

    /*  Grant access for other threads to use the crypto module.
     *  The semaphore must be posted before the callbackFxn to allow the chaining
     *  of operations.
     */
    SemaphoreP_post(&CryptoResourceCC26XX_accessSemaphore);

    Power_releaseConstraint(PowerCC26XX_DISALLOW_STANDBY);

    if (object->returnBehavior == SHA2_RETURN_BEHAVIOR_BLOCKING) {
        /* Unblock the pending task to signal that the operation is complete. */
        SemaphoreP_post(&CryptoResourceCC26XX_operationSemaphore);
    }
    else {
        object->callbackFxn((SHA2_Handle)arg0,
                             object->returnStatus,
                             object->operation,
                             object->operationType);
    }
}

/*
 *  ======== SHA2_hwiFxn ========
 */
static void SHA2_hwiFxn (uintptr_t arg0) {
    SHA2CC26X2_Object *object = ((SHA2_Handle)arg0)->object;

    if (SHA2IntStatusRaw() & SHA2_DMA_BUS_ERR) {
        object->returnStatus = SHA2_STATUS_ERROR;
    }

    SHA2IntClear(SHA2_RESULT_RDY | SHA2_DMA_IN_DONE | SHA2_DMA_BUS_ERR);

    SwiP_post(&(object->callbackSwi));
}


/*
 *  ======== SHA2_waitForAccess ========
 */
static int_fast16_t SHA2_waitForAccess(SHA2_Handle handle) {
    SHA2CC26X2_Object *object = handle->object;
    uint32_t timeout;

    /* Set to SemaphoreP_NO_WAIT to start operations from SWI or HWI context */
    timeout = object->returnBehavior == SHA2_RETURN_BEHAVIOR_BLOCKING ? object->semaphoreTimeout : SemaphoreP_NO_WAIT;

    return SemaphoreP_pend(&CryptoResourceCC26XX_accessSemaphore, timeout);
}

/*
 *  ======== SHA2_waitForResult ========
 */
static int_fast16_t SHA2_waitForResult(SHA2_Handle handle){
    SHA2CC26X2_Object *object = handle->object;

    if (object->returnBehavior == SHA2_RETURN_BEHAVIOR_POLLING) {
        /* Wait until the operation is complete and check for DMA errors. */
        if(SHA2WaitForIRQFlags(SHA2_RESULT_RDY | SHA2_DMA_BUS_ERR) & SHA2_DMA_BUS_ERR){
            object->returnStatus = SHA2_STATUS_ERROR;
        }

        /* Make sure to also clear DMA_IN_DONE as it is not cleared above
         * but will be set none-the-less.
         */
        SHA2IntClear(SHA2_RESULT_RDY | SHA2_DMA_IN_DONE | SHA2_DMA_BUS_ERR);

        /*  Grant access for other threads to use the crypto module.
         *  The semaphore must be posted before the callbackFxn to allow the chaining
         *  of operations.
         */
        SemaphoreP_post(&CryptoResourceCC26XX_accessSemaphore);

        Power_releaseConstraint(PowerCC26XX_DISALLOW_STANDBY);

        return object->returnStatus;
    }
    else if (object->returnBehavior == SHA2_RETURN_BEHAVIOR_BLOCKING) {
        SemaphoreP_pend(&CryptoResourceCC26XX_operationSemaphore, SemaphoreP_WAIT_FOREVER);

        return object->returnStatus;
    }
    else {
        return SHA2_STATUS_SUCCESS;
    }

}

/*
 *  ======== SHA2_init ========
 */
void SHA2_init(void) {
    uint_least8_t i;
    uint_fast8_t key;

    key = HwiP_disable();

    if (!isInitialized) {
        /* Call each instances' driver init function */
        for (i = 0; i < SHA2_count; i++) {
            SHA2_Handle handle = (SHA2_Handle)&(SHA2_config[i]);
            SHA2CC26X2_Object *object = (SHA2CC26X2_Object *)handle->object;
            object->isOpen = false;
        }

        isInitialized = true;
    }

    HwiP_restore(key);
}

/*
 *  ======== SHA2_open ========
 */
SHA2_Handle SHA2_open(uint_least8_t index, SHA2_Params *params) {
    SwiP_Params                 swiParams;
    SHA2_Handle                 handle;
    SHA2CC26X2_Object           *object;
    SHA2CC26X2_HWAttrs const    *hwAttrs;
    uint_fast8_t                key;

    handle = (SHA2_Handle)&(SHA2_config[index]);
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    DebugP_assert(index >= SHA2_count);

    key = HwiP_disable();

    if (!isInitialized ||  object->isOpen) {
        HwiP_restore(key);
        return NULL;
    }

    object->isOpen = true;

    HwiP_restore(key);

    /* If params are NULL, use defaults */
    if (params == NULL) {
        params = (SHA2_Params *)&SHA2_defaultParams;
    }

    DebugP_assert(params->returnBehavior == SHA2_RETURN_BEHAVIOR_CALLBACK ? params->callbackFxn : true);

    object->returnBehavior = params->returnBehavior;
    object->callbackFxn = params->callbackFxn;
    object->semaphoreTimeout = params->timeout;

    /* Create Swi object for this SHA2 peripheral */
    SwiP_Params_init(&swiParams);
    swiParams.arg0 = (uintptr_t)handle;
    swiParams.priority = hwAttrs->swiPriority;
    SwiP_construct(&(object->callbackSwi), SHA2_swiFxn, &swiParams);

    CryptoResourceCC26XX_constructRTOSObjects();

    /* Set power dependency - i.e. power up and enable clock for Crypto (CryptoResourceCC26XX) module. */
    Power_setDependency(PowerCC26XX_PERIPH_CRYPTO);

    return handle;
}

/*
 *  ======== SHA2_close ========
 */
void SHA2_close(SHA2_Handle handle) {
    SHA2CC26X2_Object         *object;

    DebugP_assert(handle);

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;

    CryptoResourceCC26XX_destructRTOSObjects();

    /* Destroy the Swi */
    SwiP_destruct(&(object->callbackSwi));

    /* Release power dependency on Crypto Module. */
    Power_releaseDependency(PowerCC26XX_PERIPH_CRYPTO);

    /* Mark the module as available */
    object->isOpen = false;
}

/*
 *  ======== SHA2_startHash ========
 */
int_fast16_t SHA2_startHash(SHA2_Handle handle, SHA2_OperationStartHash *operation) {
    SHA2CC26X2_Object *object = handle->object;
    SHA2CC26X2_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Try and obtain access to the crypto module */
    if (SHA2_waitForAccess(handle) != SemaphoreP_OK) {
        return SHA2_STATUS_RESOURCE_UNAVAILABLE;
    }

    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);

    /* If we are in SHA2_RETURN_BEHAVIOR_POLLING, we do not want an interrupt to trigger.
     * We need to disable it before kicking off the operation.
     */
    if (object->returnBehavior == SHA2_RETURN_BEHAVIOR_POLLING)  {
        IntDisable(INT_CRYPTO_RESULT_AVAIL_IRQ);
    }
    else {
        /* We need to set the HWI function and priority since the same physical interrupt is shared by multiple
         * drivers and they all need to coexist. Whenever a driver starts an operation, it
         * registers its HWI callback with the OS.
         */
        HwiP_setFunc(&CryptoResourceCC26XX_hwi, SHA2_hwiFxn, (uintptr_t)handle);
        HwiP_setPriority(INT_CRYPTO_RESULT_AVAIL_IRQ, hwAttrs->intPriority);

        IntPendClear(INT_CRYPTO_RESULT_AVAIL_IRQ);
        IntEnable(INT_CRYPTO_RESULT_AVAIL_IRQ);
    }

    object->operation.startHash = operation;
    object->operationType = SHA2_OPERATION_TYPE_START_HASH;
    object->returnStatus = SHA2_STATUS_SUCCESS;

    SHA2ComputeInitialHash(operation->message,
                           (uint32_t *)operation->intermediateDigest,
                           hashSizeTable[operation->hashSize],
                           operation->length);


    return SHA2_waitForResult(handle);

}

/*
 *  ======== SHA2_processHash ========
 */
int_fast16_t SHA2_processHash(SHA2_Handle handle, SHA2_OperationProcessHash *operation) {
    SHA2CC26X2_Object *object = handle->object;
    SHA2CC26X2_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Try and obtain access to the crypto module */
    if (SHA2_waitForAccess(handle) != SemaphoreP_OK) {
        return SHA2_STATUS_RESOURCE_UNAVAILABLE;
    }

    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);

    /* If we are in SHA2_RETURN_BEHAVIOR_POLLING, we do not want an interrupt to trigger.
     * We need to disable it before kicking off the operation.
     */
    if (object->returnBehavior == SHA2_RETURN_BEHAVIOR_POLLING)  {
        IntDisable(INT_CRYPTO_RESULT_AVAIL_IRQ);
    }
    else {
        /* We need to set the HWI function and priority since the same physical interrupt is shared by multiple
         * drivers and they all need to coexist. Whenever a driver starts an operation, it
         * registers its HWI callback with the OS.
         */
        HwiP_setFunc(&CryptoResourceCC26XX_hwi, SHA2_hwiFxn, (uintptr_t)handle);
        HwiP_setPriority(INT_CRYPTO_RESULT_AVAIL_IRQ, hwAttrs->intPriority);

        IntPendClear(INT_CRYPTO_RESULT_AVAIL_IRQ);
        IntEnable(INT_CRYPTO_RESULT_AVAIL_IRQ);
    }

    object->operation.processHash = operation;
    object->operationType = SHA2_OPERATION_TYPE_PROCESS_HASH;
    object->returnStatus = SHA2_STATUS_SUCCESS;

    SHA2ComputeIntermediateHash(operation->message,
                                (uint32_t *)operation->intermediateDigest,
                                hashSizeTable[operation->hashSize],
                                operation->length);


    return SHA2_waitForResult(handle);
}

/*
 *  ======== SHA2_finishHash ========
 */
int_fast16_t SHA2_finishHash(SHA2_Handle handle, SHA2_OperationFinishHash *operation) {
    SHA2CC26X2_Object *object = handle->object;
    SHA2CC26X2_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Try and obtain access to the crypto module */
    if (SHA2_waitForAccess(handle) != SemaphoreP_OK) {
        return SHA2_STATUS_RESOURCE_UNAVAILABLE;
    }

    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);

    /* If we are in SHA2_RETURN_BEHAVIOR_POLLING, we do not want an interrupt to trigger.
     * We need to disable it before kicking off the operation.
     */
    if (object->returnBehavior == SHA2_RETURN_BEHAVIOR_POLLING)  {
        IntDisable(INT_CRYPTO_RESULT_AVAIL_IRQ);
    }
    else {
        /* We need to set the HWI function and priority since the same physical interrupt is shared by multiple
         * drivers and they all need to coexist. Whenever a driver starts an operation, it
         * registers its HWI callback with the OS.
         */
        HwiP_setFunc(&CryptoResourceCC26XX_hwi, SHA2_hwiFxn, (uintptr_t)handle);
        HwiP_setPriority(INT_CRYPTO_RESULT_AVAIL_IRQ, hwAttrs->intPriority);

        IntPendClear(INT_CRYPTO_RESULT_AVAIL_IRQ);
        IntEnable(INT_CRYPTO_RESULT_AVAIL_IRQ);
    }

    object->operation.finishHash = operation;
    object->operationType = SHA2_OPERATION_TYPE_FINISH_HASH;
    object->returnStatus = SHA2_STATUS_SUCCESS;

    SHA2ComputeFinalHash(operation->message,
                         operation->finalDigest,
                         (uint32_t *)operation->intermediateDigest,
                         operation->totalLength,
                         operation->segmentLength,
                         hashSizeTable[operation->hashSize]);

    return SHA2_waitForResult(handle);
}

/*
 *  ======== SHA2_oneStepHash ========
 */
int_fast16_t SHA2_oneStepHash(SHA2_Handle handle, SHA2_OperationOneStepHash *operation) {
    SHA2CC26X2_Object *object = handle->object;
    SHA2CC26X2_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Try and obtain access to the crypto module */
    if (SHA2_waitForAccess(handle) != SemaphoreP_OK) {
        return SHA2_STATUS_RESOURCE_UNAVAILABLE;
    }

    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);

    /* If we are in SHA2_RETURN_BEHAVIOR_POLLING, we do not want an interrupt to trigger.
     * We need to disable it before kicking off the operation.
     */
    if (object->returnBehavior == SHA2_RETURN_BEHAVIOR_POLLING)  {
        IntDisable(INT_CRYPTO_RESULT_AVAIL_IRQ);
    }
    else {
        /* We need to set the HWI function and priority since the same physical interrupt is shared by multiple
         * drivers and they all need to coexist. Whenever a driver starts an operation, it
         * registers its HWI callback with the OS.
         */
        HwiP_setFunc(&CryptoResourceCC26XX_hwi, SHA2_hwiFxn, (uintptr_t)handle);
        HwiP_setPriority(INT_CRYPTO_RESULT_AVAIL_IRQ, hwAttrs->intPriority);

        IntPendClear(INT_CRYPTO_RESULT_AVAIL_IRQ);
        IntEnable(INT_CRYPTO_RESULT_AVAIL_IRQ);
    }

    object->operation.oneStepHash = operation;
    object->operationType = SHA2_OPERATION_TYPE_ONE_STEP_HASH;
    object->returnStatus = SHA2_STATUS_SUCCESS;

    SHA2ComputeHash(operation->message,
                    operation->digest,
                    operation->totalLength,
                    hashSizeTable[operation->hashSize]);

    return SHA2_waitForResult(handle);
}
