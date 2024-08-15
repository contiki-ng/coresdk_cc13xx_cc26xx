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
 *  ======== ADCBufChanMSP432E4.syscfg.js ========
 */

"use strict";

let adcChanNames = [
    "AIN0", "AIN1", "AIN2", "AIN3",
    "AIN3", "AIN4", "AIN5", "AIN6",
    "AIN7", "AIN8", "AIN9", "AIN10",
    "AIN11", "AIN12", "AIN12", "AIN13",
    "AIN14", "AIN15", "AIN16", "AIN17",
    "AIN18", "AIN19", "AIN20", "AIN21",
    "AIN22", "AIN23"
];

/*
 *  ======== pinmuxRequirements ========
 */
function pinmuxRequirements(inst)
{
    let requirements = [];

    if (inst.inputSource != "External Pin") {
        return (requirements);
    }

    let adcBufChan = {
        extend: inst.$ownedBy.$ownedBy.adc,
        name: "adcPin",
        displayName: "ADC Pin",
        interfaceNames: adcChanNames,
        signalTypes : ["AIN"]
    };

    requirements.push(adcBufChan);

    return (requirements);
}

/*
 *  ========= moduleInstances ========
 */
function moduleInstances(inst)
{
    let result =[];

    if (inst.mode === "Differential") {
        result.push({
            name: "differentialPin",
            displayName: "Differential Pin",
            moduleName: "/ti/drivers/adcbuf/ADCBufDiffMSP432E4"
        });
    }

    return (result);
}

/*
 *  ========= filterHardware ========
 */
function filterHardware(component)
{
    for (let sig in component.signals) {
        if (component.signals[sig].type == "AIN") {
            return (true);
        }
    }

    return (false);
}

/*
 *  ========= onInputSourceChange ========
 */
function onInputSourceChange(inst, ui)
{
    if (inst.inputSource === "External Pin") {
        ui.mode.hidden = false;
        if (ui.$hardware) {
            //ui.$hardware.hidden = false;
        }
    }
    else {
        if (ui.$hardware) {
            //ui.$hardware.hidden = true;
            inst.$hardware = null;
        }
        ui.mode.hidden = true;
        inst.mode = "Non Differential";
    }
}

/*
 *  ======== exports ========
 */
exports = {
    config: [
        {
            name: "$name",
            defaultPrefix: "ADCBUF_CHANNEL_",
            description: "C identifier used to identify this ADCBuf channel.",
                hidden: false
        },
        {
            name: "referenceVoltage",
            displayName: "Reference Voltage",
            description: "Reference voltage in microvolts.",
            default: 3300000
        },
        {
            name: "inputSource",
            displayName: "Input Source",
            description: "Specifies the channel input source.",
            default: "External Pin",
            options: [
                {
                    name: "External Pin",
                    description: "This channel will sample an external analog"
                        + " signal"
                },
                {
                    name: "Internal Temperature",
                    description: "This channel will sample the internal"
                        + " temperature analog signal"
                }
            ],
            onChange : onInputSourceChange
        },
        {
            name: "mode",
            displayName: "Mode",
            description: "Specifies the ADC channel's operation mode",
            default: "Non Differential",
            options: [
                {
                    name: "Non Differential",
                    description: "Sample a single analog signal with this"
                    +   " channel"
                },
                {
                    name: "Differential",
                    description: "Measure the voltage potential between 2"
                    +   " analog signals."
                }
            ]
        }
    ],

    displayName: "ADCBuf_Channel",

    moduleInstances: moduleInstances,

    /* override generic requirements with  device-specific reqs (if any) */
    pinmuxRequirements: pinmuxRequirements,

    filterHardware: filterHardware
};
