/*
 * Copyright (c) 2018-2019 Texas Instruments Incorporated - http://www.ti.com
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
 *  ======== ADCMSP432E4.syscfg.js ========
 */

"use strict";

/* get Common /ti/drivers utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

/*
 *  ======== devSpecific ========
 *  Device-specific extensions to be added to base ADC configuration
 */
let devSpecific = {
    config: [
        {
            name: "referenceSource",
            displayName: "Reference Source",
            default: "Internal",
            description: "Specifies the ADC's reference voltage source.",
            options: [
                { name: "Internal" },
                { name: "External" }
            ]
        },
        {
            name: "resolution",
            displayName: "Resolution",
            description: "Specifies the ADC's resolution",
            hidden: true,
            default: "12 Bits",
            options: [
                { name: "12 Bits" }
            ]
        },
        {
            name: "sequencer",
            displayName: "Sequencer",
            description: "Specifies the ADC hardware sequencer(s) to use.",
            longDescription: "Each ADC module contains four programmable"
                + " sequencers allowing the sampling of multiple analog input"
                + " sources without controller intervention.",
            default: 0,
            options: [
                { name: 0 },
                { name: 1 },
                { name: 2 },
                { name: 3 }
            ]
        }
    ],

    /* override generic requirements with device-specific reqs (if any) */
    pinmuxRequirements: pinmuxRequirements,

    modules: Common.autoForceModules(["Board", "Power"]),

    templates: {
        boardc: "/ti/drivers/adc/ADCMSP432E4.Board.c.xdt",
        boardh: "/ti/drivers/adc/ADC.Board.h.xdt"
    },

    _getPinResources: _getPinResources
};

/*
 *  ======== _getPinResources ========
 */
function _getPinResources(inst)
{

    let pinPort = inst.adc.adcPin.$solution.devicePinName.substring(0, 2);
    let pinNum = inst.adc.adcPin.$solution.devicePinName.substring(2, 4);
    let pin = pinPort + pinNum;

    if (inst.$hardware && inst.$hardware.displayName) {
        pin += ", " + inst.$hardware.displayName;
    }

    return (pin);
}

/*
 *  ======== pinmuxRequirements ========
 *  Returns peripheral pin requirements of the specified instance
 *
 *  @param inst    - a fully configured ADC instance
 *
 *  @returns req[] - an array of pin requirements needed by inst
 */
function pinmuxRequirements(inst)
{
    let adc = {
        name: "adc",
        displayName: "ADC Peripheral",
        interfaceName: "ADC",
        canShareWith: "ADC",
        resources: [
            {
                name: "adcPin",
                hidden: false,
                displayName: "ADC Pin",
                interfaceNames: [
                    "AIN0", "AIN1", "AIN2", "AIN3",
                    "AIN3", "AIN4", "AIN5", "AIN6",
                    "AIN7", "AIN8", "AIN9", "AIN10",
                    "AIN11", "AIN12", "AIN12", "AIN13",
                    "AIN14", "AIN15", "AIN16", "AIN17",
                    "AIN18", "AIN19", "AIN20", "AIN21",
                    "AIN22", "AIN23"
                ]
            }
        ],
        signalTypes: { adcPin: ["AIN"] }
    };

    return ([adc]);
}

/*
 *  ======== extend ========
 *  Extends a base exports object to include any device specifics
 *
 *  This function is invoked by the generic ADC module to
 *  allow us to augment/override as needed.
 */
function extend(base)
{
    /* display which driver implementation can be used */
    base = Common.addImplementationConfig(base, "ADC", null,
        [{name: "ADCMSP432E4"}], null);

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
    /* required function, called by base ADC module */
    extend: extend
};
