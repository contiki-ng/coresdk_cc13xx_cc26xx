/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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
#include <stddef.h>
#include <stdbool.h>

#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/dpl/ClockP.h>

#include <ti/drivers/Timer.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/drivers/timer/TimerCC26XX.h>

void TimerCC26XX_close(Timer_Handle handle);
int_fast16_t TimerCC26XX_control(Timer_Handle handle,
     uint_fast16_t cmd, void *arg);
uint32_t TimerCC26XX_getCount(Timer_Handle handle);
void TimerCC26XX_init(Timer_Handle handle);
Timer_Handle TimerCC26XX_open(Timer_Handle handle, Timer_Params *params);
int32_t TimerCC26XX_setPeriod(Timer_Handle handle, Timer_PeriodUnits periodUnits, uint32_t period);
int32_t TimerCC26XX_start(Timer_Handle handle);
void TimerCC26XX_stop(Timer_Handle handle);

/* Internal callback function */
static void TimerCC26XX_callbackfxn(GPTimerCC26XX_Handle gptHandle, GPTimerCC26XX_IntMask intMask);

/* Function table for TimerCC26XX implementation */
const Timer_FxnTable TimerCC26XX_fxnTable = {
    .closeFxn     = TimerCC26XX_close,
    .openFxn      = TimerCC26XX_open,
    .startFxn     = TimerCC26XX_start,
    .stopFxn      = TimerCC26XX_stop,
    .initFxn      = TimerCC26XX_init,
    .getCountFxn  = TimerCC26XX_getCount,
    .controlFxn   = TimerCC26XX_control,
    .setPeriodFxn = TimerCC26XX_setPeriod
};

/*
 *  ======== TimerCC26XX_close ========
 */
void TimerCC26XX_close(Timer_Handle handle)
{
    TimerCC26XX_Object *object = handle->object;

    /* Unregister Interrupts and Callback */
    GPTimerCC26XX_unregisterInterrupt(object->gptHandle);

    /* Destruct semaphore */
    if (object->semHandle != NULL) {
        SemaphoreP_destruct(&(object->semStruct));
        object->semHandle = NULL;
    }

    /* Close and delete gptHandle */
    GPTimerCC26XX_close(object->gptHandle);
    object->gptHandle = NULL;

    /* Clear isOpen flag */
    object->isOpen = false;
}

/*
 *  ======== TimerCC26XX_control ========
 */
int_fast16_t TimerCC26XX_control(Timer_Handle handle,
        uint_fast16_t cmd, void *arg)
{
    return (Timer_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== TimerCC26XX_getCount ========
 */
uint32_t TimerCC26XX_getCount(Timer_Handle handle)
{
    TimerCC26XX_Object const *object = handle->object;
    uint32_t count;

    count = GPTimerCC26XX_getValue(object->gptHandle);

    return count;
}

/*
 *  ======== TimerCC26XX_callbackfxn ========
 */
void TimerCC26XX_callbackfxn(GPTimerCC26XX_Handle gptHandle, GPTimerCC26XX_IntMask intMask)
{
    Timer_Handle handle;
    /* Get Timer Handle from arg in GPTimerCC26XX_Object */
    handle = (Timer_Handle)GPTimerCC26XX_getArg(gptHandle);
    TimerCC26XX_Object *object = handle->object;

    /* Callback not created when using Timer_FREE_RUNNING */
    if (object->mode != Timer_CONTINUOUS_CALLBACK) {
        TimerCC26XX_stop(handle);
    }
    if (object->mode != Timer_ONESHOT_BLOCKING) {
        object->callBack(handle, Timer_STATUS_SUCCESS);
    }
}

/*
 *  ======== TimerCC26XX_init ========
 */
void TimerCC26XX_init(Timer_Handle handle)
{
    return;
}

/*
 *  ======== TimerCC26XX_open ========
 */
Timer_Handle TimerCC26XX_open(Timer_Handle handle, Timer_Params *params)
{
    TimerCC26XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    TimerCC26XX_Object *object = handle->object;
    GPTimerCC26XX_Params       gptParams;
    int_fast16_t status;

    /* Check if timer is already open */
    uint32_t key = HwiP_disable();
    if (object->isOpen)
    {
        HwiP_restore(key);
        return (NULL);
    }
    object->isOpen = true;
    HwiP_restore(key);

    /* Check for valid parameters */
    if (((params->timerMode == Timer_ONESHOT_CALLBACK ||
          params->timerMode == Timer_CONTINUOUS_CALLBACK) &&
          params->timerCallback == NULL) ||
          params->period == 0) {

        object->isOpen = false;
        return (NULL);
    }

    object->mode = params->timerMode;
    object->period = params->period;
    object->callBack = params->timerCallback;
    object->isRunning = false;

    /* Initialize gptParams to default values */
    GPTimerCC26XX_Params_init(&gptParams);

    /* Convert Timer mode to GPT mode */
    switch (params->timerMode) {
        case Timer_ONESHOT_CALLBACK:
        case Timer_ONESHOT_BLOCKING:
            gptParams.mode = GPT_MODE_ONESHOT;
            break;

        case Timer_CONTINUOUS_CALLBACK:
        case Timer_FREE_RUNNING:
        default:
            gptParams.mode = GPT_MODE_PERIODIC;
            break;
    }

    /* Convert Timer width to GPT width */
    switch (hwAttrs->subTimer) {
        case TimerCC26XX_timer32:
            gptParams.width = GPT_CONFIG_32BIT;
            break;

        case TimerCC26XX_timer16A:
        case TimerCC26XX_timer16B:
        default:
            gptParams.width = GPT_CONFIG_16BIT;
            break;
    }

    /* Pass a GPTimer index and params to GPTimer open call */
    object->gptHandle = GPTimerCC26XX_open(hwAttrs->gpTimerUnit, &gptParams);

    /* Check if returned gptHandle is null */
    if (object->gptHandle == NULL) {
        object->isOpen = false;
        return (NULL);
    }

    if (params->timerMode != Timer_FREE_RUNNING) {
        /* Set Timer Period and Units */
        status = Timer_setPeriod(handle, params->periodUnits, object->period);
        if (status != Timer_STATUS_SUCCESS) {
            TimerCC26XX_close(handle);
            return (NULL);
        }

        /* Create the semaphore for blocking mode */
        if (params->timerMode == Timer_ONESHOT_BLOCKING) {
            object->semHandle = SemaphoreP_constructBinary(&(object->semStruct), 0);

            if (object->semHandle == NULL) {
                TimerCC26XX_close(handle);
                return (NULL);
            }
        }

        /* Set custom arg in GPTimerCC26XX_Object to Timer handle */
        GPTimerCC26XX_setArg(object->gptHandle, (void *)handle);

        /* Register Interrupt */
        GPTimerCC26XX_registerInterrupt(object->gptHandle, TimerCC26XX_callbackfxn, GPT_INT_TIMEOUT);
    }

    return (handle);
}

/*
 *  ======== TimerCC26XX_setPeriod ========
 */
int32_t TimerCC26XX_setPeriod(Timer_Handle handle, Timer_PeriodUnits periodUnits, uint32_t period)
{
    TimerCC26XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    TimerCC26XX_Object *object = handle->object;
    ClockP_FreqHz              clockFreq;

    /* Check for valid period */
    ClockP_getCpuFreq(&clockFreq);

    if (periodUnits == Timer_PERIOD_US) {
        /* Checks if the calculated period will fit in 32-bits */
        if (period >= ((uint32_t) ~0) / (clockFreq.lo / 1000000)) {

            return (Timer_STATUS_ERROR);
        }

        period = period * (clockFreq.lo / 1000000);
    }
    else if (periodUnits == Timer_PERIOD_HZ) {
        /* If period > clockFreq */
        if ((period = clockFreq.lo / period) == 0) {

            return (Timer_STATUS_ERROR);
        }
    }

    /* If using a half timer */
    if (hwAttrs->subTimer != TimerCC26XX_timer32) {
            /* 24-bit resolution for the half timer */
            if (period >= (1 << 24)) {

                return (Timer_STATUS_ERROR);
            }
    }

    object->period = period;

    /*Set period */
    GPTimerCC26XX_setLoadValue(object->gptHandle, object->period);

    return (Timer_STATUS_SUCCESS);
}

/*
 *  ======== TimerCC26XX_start ========
 */
int32_t TimerCC26XX_start(Timer_Handle handle)
{
    TimerCC26XX_Object *object = handle->object;

    /* Check if timer is already running */
    uint32_t key = HwiP_disable();
    if (object->isRunning) {
        HwiP_restore(key);
        return(Timer_STATUS_ERROR);
    }
    object->isRunning = true;
    GPTimerCC26XX_start(object->gptHandle);
    HwiP_restore(key);

    /* Pend on semaphore in blocking mode */
    if (object->mode == Timer_ONESHOT_BLOCKING) {
        /* Pend forever, ~0 */
        SemaphoreP_pend(object->semHandle, SemaphoreP_WAIT_FOREVER);
    }

    return (Timer_STATUS_SUCCESS);
}

/*
 *  ======== TimerCC26XX_stop ========
 */
void TimerCC26XX_stop(Timer_Handle handle)
{
    TimerCC26XX_Object *object = handle->object;
    bool               flag = false;

    uint32_t key = HwiP_disable();
    if (object->isRunning) {
        object->isRunning = false;
        /* Post the Semaphore when called from the callback */
        if (object->mode == Timer_ONESHOT_BLOCKING) {
            flag = true;
        }

        GPTimerCC26XX_stop(object->gptHandle);
    }
    HwiP_restore(key);

    if (flag) {
        /* Post semaphore in oneshot blocking mode */
        SemaphoreP_post(object->semHandle);
    }
}
