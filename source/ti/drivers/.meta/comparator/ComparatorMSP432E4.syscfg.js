/*
 * Copyright (c) 2019 Texas Instruments Incorporated - http://www.ti.com
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
 *  ========= ComparatorMSP432E4.syscfg.js =======
 */

"use strict";

/* get Common /ti/drivers utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

let intPriority = Common.newIntPri()[0];
intPriority.name = "interruptPriority";
intPriority.displayName = "Interrupt Priority";
intPriority.description = "Analog Comparator peripheral interrupt priority";

/*
 *  ======= devSpecific =====
 *  Device-specific extensions to be addeed to base Comparator configuration
 */
let devSpecific = {
    config: [
        {
            name: "positiveInputSource",
            displayName: "Positive Input Source",
            default: "Internal Reference",
            description: "Specifies Comparator's positive terminal input source.",
            options: [
                { name: "Internal Reference" },
                { name: "Comparator Pin" },
                { name: "Shared Comparator Pin" }
            ]
        },
        {
            name: "triggerLevel",
            displayName: "Trigger Level",
            default: "None",
            description: "Specifies what Comparator output will trigger an ADC conversion.",
            options: [
                { name: "None" },
                { name: "Output High" },
                { name: "Output Low" },
                { name: "Rising Edge" },
                { name: "Falling Edge" },
                { name: "Both Edges" }
            ]
        },
        intPriority
    ],

    /*
     * Need a static comparator module to allow the shared pin configuration
     * and synchronize the shared internal reference configuration
     */
    moduleStatic: {
        name: "comparatorGlobal",
        config: [
            {
                name: "referenceLevel",
                displayName: "Reference Level",
                default: "0V",
                description: "Specifies the voltage level for the shared internal reference.",
                options: [
                    { name: "0V" },
                    { name: "0.1375V" },
                    { name: "0.275V" },
                    { name: "0.4125V" },
                    { name: "0.55V" },
                    { name: "0.6875V" },
                    { name: "0.825V" },
                    { name: "0.928125V" },
                    { name: "0.9625V" },
                    { name: "1.03125V" },
                    { name: "1.134375V" },
                    { name: "1.1V" },
                    { name: "1.2375V" },
                    { name: "1.340625V" },
                    { name: "1.375V" },
                    { name: "1.44375V" },
                    { name: "1.5125V" },
                    { name: "1.546875V" },
                    { name: "1.65V" },
                    { name: "1.753125V" },
                    { name: "1.7875V" },
                    { name: "1.85625V" },
                    { name: "1.925V" },
                    { name: "1.959375V" },
                    { name: "2.0625V" },
                    { name: "2.165625V" },
                    { name: "2.26875V" },
                    { name: "2.371875V" }
                ]
            },
            {
                name: "sharedPinEnable",
                displayName: "Enable Shared Pin",
                default: false,
                description: "Selects whether the shared pin option will be used, \
                must be enabled in conjunction with comparator positive input",
                longDescription: `
The shared comparator pin option allows multiple comparators to
use the positive input pin of the Comparator0 Analog Comparator Peripheral
rather than their own dediated pin. This option will use
fewer pins when multiple comparators should
have the same input signal.
`
            }
        ],
        /* Set of pinmuxRequirements for the shared, static instance */
        pinmuxRequirements: staticPinmuxRequirements
    },

    /* Override default pinmuxRequirements with device-specific requirements */
    pinmuxRequirements: pinmuxRequirements,

    templates: {
        boardc: "/ti/drivers/comparator/ComparatorMSP432E4.Board.c.xdt",
        boardh: "/ti/drivers/comparator/Comparator.Board.h.xdt"
    },

    _getPinResources: _getPinResources,

    validate: validate

};

/*
 *  ========= _getPinResources ===========
 *
 *
 *
 */
function _getPinResources(inst)
{
    let pin;
    /* Default to this, otherwise will be overwritten */
    let positivePin = "Internal Reference";
    let negativePin = "Unassigned";
    let outputPin = "Unassigned";

    if(inst.comparator) {
        if(inst.comparator.positivePin) {
            positivePin = inst.comparator.positivePin.$solution.devicePinName;
        }
        if(inst.comparator.negativePin) {
            negativePin = inst.comparator.negativePin.$solution.devicePinName;
        }
        if(inst.comparator.outputPin) {
            outputPin = inst.comparator.outputPin.$solution.devicePinName;
        }
        pin = "\nPositive Terminal: " + positivePin + "\nNegative terminal: "
                + negativePin + "\nOutput Pin: " + outputPin;
        if(inst.$hardware && inst.$hardware.displayName) {
            pin += ", " + inst.$hardware.displayName;
        }
    }

    if(inst.$module.$static.sharedComparator) {
        if(inst.positiveInputSource === "Shared Comparator Pin"
            && inst.$module.$static.sharedComparator.sharedPin) {
            positivePin = inst.$module.$static.sharedComparator.sharedPin.$solution.devicePinName;
        }
        pin = "\nPositive Terminal: " + positivePin + "\nNegative terminal: "
                + negativePin + "\nOutput Pin: " + outputPin;
    }

    return (pin);
}

/*
 *  ========= pinmuxRequirements =========
 *  Returns a peripheral pin requirements of the specified instance
 *
 *  @param inst     - a fully configured Comparator instance
 *
 *  @returns req[]  - an array of pin requirements needed by inst
 */
function pinmuxRequirements(inst)
{
    let comparator = {
        name: "comparator",
        displayName: "Analog Comparator Peripheral",
        interfaceName: "Comparator",
        canShareWith: "Comparator",
        resources: [],
        signalTypes: {
            "posPin" : "AIN",
            "negPin" : "AIN"
        }
    };

    /* Only request positive pin if needed */
    if(inst.positiveInputSource === "Comparator Pin") {
        comparator.resources.push(
            {
                name: "positivePin",
                hidden: false,
                displayName: "Positive Pin",
                interfaceNames: [
                    "+"
                ]
            }
        );
    }

    if(inst.outputEnable === true) {
        /* Output pin is always the last element */
        comparator.resources.push(
            {
                name: "outputPin",
                hidden: false,
                displayName: "Output Pin",
                interfaceNames: [
                    "O"
                ]
            }
        );
    }

    /* Negative terminal not optional */
    comparator.resources.push(
        {
            name: "negativePin",
            hidden: false,
            displayName: "Negative Pin",
            interfaceNames: [
                "-"
            ]
        }
    );


    return ([comparator]);
}

/*
 *  ======== staticPinmuxRequirements =========
 *  Returns peripheral requirements for inst, similar to pinmuxRequirements
 *
 *  @param inst     - a fully configured Comparator instance
 *
 *  @returns req[]  - an array of pin requirements needed for inst
 */
function staticPinmuxRequirements(inst)
{

    if(inst.sharedPinEnable === false)
    {
        return ([]);
    }

    let comparator = {
        name: "sharedComparator",
        displayName: "Shared Pin Base Module",
        description: "The shared pin configuration is based off of the Comparator 0 module's positive pin",
        interfaceName: "Comparator",
        canShareWith: "Comparator",
        filter: shareFilter,
        resources: [
            {
                name: "sharedPin",
                displayName: "Shared Input Pin",
                filter: pinFilter,
                interfaceNames: [
                    "+"
                ]
            }
        ]
    };

    return ([comparator]);
}

/*
 *  ========= pinFilter ===========
 *  Utility function to filter the correct shared pin
 *  to the static module instance, used in pinmuxStatic
 */
function pinFilter(devicePin, peripheralPin) {

    if(peripheralPin.name === "C0+")
    {
        return (true);
    }

    return (false);
}

/*
 *  ========= shareFilter =========
 *  Utility function to filter the correct Comparator
 *  interface for the shared configuration, used in
 *  pinmuxStatic
 */
function shareFilter(iface, peripheralPin)
{
    if(iface.name === "Comparator0")
    {
        return (true);
    }

    return (false);
}

/*
 *  ======== validate =========
 *
 *
 */
function validate(inst, validation)
{

    if(inst.positiveInputSource === "Shared Comparator Pin"
        && inst.$module.$static.sharedPinEnable === false)
    {
        Common.logError(validation, inst, "positiveInputSource", "Comparator pin sharing requires global enable");
    }
}

/*
 *  ======= extend ========
 *  Extends a base exports object to include any device specifics
 *
 *  This function is invoked by the generic Comparator module to
 *  allow us to augment/override as needed.
 */
function extend(base)
{
    /* display which driver implementation can be used */
    devSpecific = Common.addImplementationConfig(devSpecific, "Comparator", null,
        [{name: "ComparatorMSP432E4"}], null);

    let result = Object.assign({}, base, devSpecific);

    result.config = base.config.concat(devSpecific.config);

    return (result);

}

/*
 *  ======= exports ========
 *  Export device-specific extensions to base exports
 */
exports = {
    /* required function, called by Comparator module */
    extend: extend
};
