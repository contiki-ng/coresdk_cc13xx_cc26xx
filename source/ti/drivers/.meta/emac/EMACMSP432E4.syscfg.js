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
 *  ======== EMACMSP432E4.syscfg.js ========
 */

"use strict";

/* get Common /ti/drivers utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

let intPriority = Common.newIntPri()[0];
intPriority.name = "interruptPriority";
intPriority.displayName = "Interrupt Priority";

/*
 *  ======== devSpecific ========
 *  Device-specific extensions to be added to base EMAC configuration
 */
let devSpecific = {
    config: [
        intPriority,
        {
            name: "macAddress",
            displayName: "MAC Address",
            description: "MAC Address used by the EMAC driver.",
            longDescription: "When the MAC address is the default,"
                + " `ff:ff:ff:ff:ff:ff`, the EMAC driver reads a MAC address"
                + " value stored in flash. To override this behavior,"
                + " manually set the MAC address.",
            placeholder: "00:00:00:00:00:00",
            textType: "mac_address",
            default: "ff:ff:ff:ff:ff:ff"
        }
    ],

    pinmuxRequirements: pinmuxRequirements,

    filterHardware: filterHardware,

    templates : {
        boardc: "/ti/drivers/emac/EMACMSP432E4.Board.c.xdt",
        "/ti/utils/rov/syscfg_c.rov.xs.xdt": "/ti/drivers/emac/EMACMSP432E4.rov.js"
    },

    _getPinResources: _getPinResources
};

/*
 *  ======== _getPinResources ========
 */
function _getPinResources(inst)
{
    let pin;

    if (inst.emac) {
        let led0Pin = inst.emac.led0Pin.$solution.devicePinName;
        let led1Pin = inst.emac.led1Pin.$solution.devicePinName;
        pin = "\nLED 0: " + led0Pin + "\nLED 1: " + led1Pin;

        if (inst.$hardware && inst.$hardware.displayName) {
            pin += "\n" + inst.$hardware.displayName;
        }
    }

    return (pin);
}

/*
 *  ======== pinmuxRequirements ========
 *  Return peripheral pin requirements as a function of config
 */
function pinmuxRequirements(inst)
{
    let emac = {
        name: "emac",
        displayName: "EMAC Peripheral",
        interfaceName: "ETHERNET",

        resources: [
            {
                name: "led0Pin",
                displayName: "LED 0 Pin",
                description: "Ethernet Link OK",
                interfaceNames: ["EN0LED0"]
            },
            {
                name: "led1Pin",
                displayName: "LED 1 Pin",
                description: "Ethernet TX/RX Activity",
                interfaceNames: ["EN0LED1"]
            }
        ],
        signalTypes: {
            led0Pin: ["EN0LED0"],
            led1Pin: ["EN0LED1"]
        }
    };

    return ([emac]);
}

/*
 *  ======== filterHardware ========
 */
function filterHardware(component)
{
    return (Common.typeMatches(component.type, ["EthernetLED"]));
}

/*
 *  ======== extend ========
 *  Extends a base exports object to include any device specifics
 *
 *  This function is invoked by the generic module to
 *  allow us to augment and override as needed.
 */
function extend(base)
{
    /* display which driver implementation can be used */
    base = Common.addImplementationConfig(base, "EMAC", null,
        [{name: "EMACMSP432E4"}], null);

    return (Object.assign({}, base, devSpecific));
}

/*
 *  ======== exports ========
 *  Export device-specific extensions to base exports
 */
exports = {
    extend: extend
};
