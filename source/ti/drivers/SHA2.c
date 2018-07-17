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
/*
 *  ======== SHA2.c ========
 *
 *  This file contains default values for the SHA2_Params struct.
 *
 */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <ti/drivers/SHA2.h>
#include <ti/drivers/dpl/SemaphoreP.h>

const SHA2_Params SHA2_defaultParams = {
    .returnBehavior = SHA2_RETURN_BEHAVIOR_BLOCKING,
    .callbackFxn = NULL,
    .timeout = SemaphoreP_WAIT_FOREVER,
    .custom = NULL,
};

/*
 *  ======== SHA2_Params_init ========
 */
void SHA2_Params_init(SHA2_Params *params){
    *params = SHA2_defaultParams;
}

/*
 *  ======== SHA2_OperationOneStepHash_init ========
 */
void SHA2_OperationOneStepHash_init(SHA2_OperationOneStepHash *operation){
    memset(operation, 0x00, sizeof(SHA2_OperationOneStepHash));
}

/*
 *  ======== SHA2_OperationStartHash_init ========
 */
void SHA2_OperationStartHash_init(SHA2_OperationStartHash *operation){
    memset(operation, 0x00, sizeof(SHA2_OperationStartHash));
}

/*
 *  ======== SHA2_OperationProcessHash_init ========
 */
void SHA2_OperationProcessHash_init(SHA2_OperationProcessHash *operation){
    memset(operation, 0x00, sizeof(SHA2_OperationProcessHash));
}

/*
 *  ======== SHA2_OperationFinishHash_init ========
 */
void SHA2_OperationFinishHash_init(SHA2_OperationFinishHash *operation){
    memset(operation, 0x00, sizeof(SHA2_OperationFinishHash));
}
