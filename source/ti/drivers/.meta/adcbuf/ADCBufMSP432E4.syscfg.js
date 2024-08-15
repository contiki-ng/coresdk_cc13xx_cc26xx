/*
 * Copyright (c) 2018-2020 Texas Instruments Incorporated - http://www.ti.com
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
 *
 */

/*
 *  ======== ADCBufMSP432E4.syscfg.js ========
 */

"use strict";

/* get Common /ti/drivers utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

let intPriority = Common.newIntPri()[0];
intPriority.name = "interruptPriority";
intPriority.displayName = "Interrupt Priority";

/* Sequencer 0-3 support 8, 4, 4, and 1 channel(s) respectively */
var MAX_SEQ_CHANS = [8, 4, 4, 1];

/*
 *  ======== devSpecific ========
 *  Device-specific extensions to be added to base ADCBuf configuration
 */
let devSpecific = {
    config: [
        {
            name: "sequencers",
            displayName: "Sequencers",
            description: "Selects the ADC sequencers for which channels will"
                + " be configured.",
            longDescription:"Each sequencer can support 8, 4, 4, and 1"
                + " channel(s) respectively.",
            default: [0],
            options: [
                {
                    name: 0,
                    description: "Sequencer 0 can support up to 8 channels."
                 },
                {
                    name: 1,
                    description: "Sequencer 1 can support up to 4 channels."
                },
                {
                    name: 2,
                    description: "Sequencer 2 can support up to 4 channels."
                },
                {
                    name: 3,
                    description: "Sequencer 3 can support 1 channel."
                }
            ]
        },
        {
            name: "modulePhase",
            displayName: "Module Phase",
            description: "Specifies the module phase in degrees.",
            longDescription:`
Specifies the phase delay between a trigger and the start of a sequence for
the ADC module. By selecting a different phase delay for a pair of ADC
modules (such as __0__ and __180__) and having each ADC module sample the
same analog input, it is possible to increase the sampling rate of the analog
input (with samples N, N+2, N+4, and so on, coming from the first ADC and
samples N+1, N+3, N+5, and so on, coming from the second ADC). The ADC module
has a single phase delay that is applied to all sample sequences within that
module.
`,
            default: 0,
            options : [
                {name: 0},   {name: 22.5},  {name: 45},  {name: 67.5},
                {name: 90},  {name: 112.5}, {name: 135}, {name: 157.5},
                {name: 180}, {name: 202.5}, {name: 225}, {name: 247.5},
                {name: 270}, {name: 292.5}, {name: 315}, {name: 337.5}
            ]
        },
        {
            name: "referenceSource",
            displayName: "Reference Source",
            default: "Internal",
            description: "Specifies the ADC reference voltage source.",
            options: [
                {
                    name: "Internal",
                    description: "Internal reference voltage source of 3V."
                },
                {
                    name: "External",
                    description: "External reference voltage of 3V supplied to"
                        + " the AVREF pin"
                }
            ]
        },
        {
            name: "enableDMA",
            displayName: "Enable DMA",
            description: "Specifies if the DMA is used.",
            default: true
        },
        {
            name: "enableTimer",
            displayName: "Enable Timer",
            description: "Enabling the timer allows sequencers to use a"
                + " timer peripheral as a trigger source",
            longDescription:`
When __true__, trigger ADC samples using a general purpose timer. When using
both ADC peripherals, both must be initialized to use the same general purpose
timer. In which case the __Module Phase__ is effective.
`,
            default: true
        },

        intPriority

    ],

    moduleInstances: moduleInstances,

    /* override generic requirements with  device-specific reqs (if any) */
    pinmuxRequirements: pinmuxRequirements,

    templates: {
        boardc: "/ti/drivers/adcbuf/ADCBufMSP432E4.Board.c.xdt",
        boardh: "/ti/drivers/adcbuf/ADCBufMSP432E4.Board.h.xdt"
    },

    _getPinResources : _getPinResources
};

/*
 *  ======== _getPinResources ========
 * Assume inst is a channel
 */
function _getPinResources(inst)
{
    let pin;

    if (inst.inputSource && inst.inputSource === "Internal Temperature") {
        return ("Temperature Sensor, ADC_CTL_TS");
    }

   if (inst.adcPin) {
       pin = inst.adcPin.$solution.devicePinName;
   }

    if (inst.$hardware && inst.$hardware.displayName) {
        pin = "\n" + pin + ", " + inst.$hardware.displayName;
    }

   if (inst.differentialPin) {
       let adcPin = inst.differentialPin.adcDifferentialPin.$solution.devicePinName;
       pin = "\n" + pin + "\nDifferential Pin: " + adcPin;
   }

   return (pin);
}

/*
 *  ======== pinmuxRequirements ========
 */
function pinmuxRequirements(inst)
{
    let requirements = [];

    let timer = {
        name: "timer",
        displayName: "Trigger Source Timer",
        interfaceName: "Timer",
        resources: []
    };

    let adc = {
        name: "adc",
        displayName: "ADC",
        interfaceName: "ADC",
        resources: []
    };

    if (inst.enableTimer === true) {
        requirements.push(timer);
    }

    requirements.push(adc);

    return (requirements);
}

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst)
{
    let result = [];
    let triggerSource = "Timer";

    if (inst.enableTimer === false) {
        triggerSource = "Auto";
    }

    /* Create a Sequence instance for each of the unique sequencers. */
    for (let i in inst.sequencers) {
        let sequencer = inst.sequencers[i];
        result.push({
            name: "sequencer" + sequencer,
            displayName: "Sequencer " + sequencer + " ",
            moduleName: "/ti/drivers/adcbuf/ADCBufSeqMSP432E4",
            args: {
                maxChans: MAX_SEQ_CHANS[sequencer],
                number: parseInt(sequencer),
                triggerSource: triggerSource,
                enableDMA: inst.enableDMA
            }
        });
    }

    return (result);
}

/*
 *  ======== extend ========
 */
function extend(base)
{
    /* display which driver implementation can be used */
    base = Common.addImplementationConfig(base, "ADCBuf", null,
        [{name: "ADCBufMSP432E4"}], null);

    /* merge and overwrite base module attributes */
    let result = Object.assign({}, base, devSpecific);

    /* concatenate device-specific configs */
    result.config = base.config.concat(devSpecific.config);

    return (result);
}

/*
 *  ======== exports ========
 *  Export device-specific extensions to base exports
 */
exports = {
    /* required function, called by base ADCBuf module */
    extend: extend
};
