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
 *  ======== SPICC26XXDMA.syscfg.js ========
 */

"use strict";

/* get Common /ti/drivers utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

let intPriority = Common.newIntPri()[0];
intPriority.name = "dmaInterruptPriority";
intPriority.displayName = "DMA Interrupt Priority";

let swiPriority = Common.newSwiPri();
swiPriority.name = "softwareInterruptPriority";
swiPriority.displayName = "Software Interrupt Priority";
swiPriority.description = "SPI Software Interrupt Priority";

/*
 *  ======== devSpecific ========
 *  Device-specific extensions to be added to base SPI configuration
 */
let devSpecific = {

    config: [
        intPriority,
        swiPriority
    ],

    /* override generic requirements with  device-specific reqs (if any) */
    pinmuxRequirements: pinmuxRequirements,

    templates: {
        boardc: "/ti/drivers/spi/SPICC26XXDMA.Board.c.xdt",
        boardh: "/ti/drivers/spi/SPI.Board.h.xdt"
    },

    /* PIN instances */
    moduleInstances: moduleInstances,

    _getPinResources: _getPinResources
};

/*
 *  ======== _getPinResources ========
 */
function _getPinResources(inst)
{
    let pin;
    let mosi = "Unassigned";
    let miso = "Unassigned";
    let sclk;
    let ss;

    if (inst.spi) {
        if (inst.spi.mosiPin) {
            mosi = inst.spi.mosiPin.$solution.devicePinName.replace("_", "");
        }
        if (inst.spi.misoPin) {
            miso = inst.spi.misoPin.$solution.devicePinName.replace("_", "");
        }

        pin = "\nMOSI: " + mosi + "\nMISO: " + miso;

        if (inst.spi.sclkPin) {
            sclk = inst.spi.sclkPin.$solution.devicePinName.replace("_", "");
            pin += "\nSCLK: " + sclk;
        }
        if (inst.spi.ssPin) {
            ss = inst.spi.ssPin.$solution.devicePinName.replace("_", "");
            pin += "\nSS: " + ss;
        }

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
    let misoRequired = false;
    let mosiRequired = false;
    let txRequired   = true;
    let rxRequired   = true;

    switch (inst.duplex) {
        case "Full":
            misoRequired = true;
            mosiRequired = true;
            break;
        case "Master TX Only":
            misoRequired = false;
            mosiRequired = true;
            rxRequired   = false;
            break;
        case "Slave RX Only":
            misoRequired = false;
            mosiRequired = true;
            txRequired   = false;
            break;
        case "Master RX Only":
            misoRequired = true;
            mosiRequired = false;
            txRequired   = false;
            break;
        case "Slave TX Only":
            misoRequired = true;
            mosiRequired = false;
            rxRequired   = false;
            break;
    }

    let spi = {
        name: "spi",
        displayName: "SPI Peripheral",
        interfaceName: "SSI",
        canShareWith: "SPI",
        resources: [
            {
                name: "sclkPin",
                displayName: "SCLK Pin",
                description: "SPI Serial Clock",
                interfaceNames: ["CLK"]
            }
        ]
    };

    if (misoRequired) {
        spi.resources.push({
            name: "misoPin",
            displayName: "MISO Pin",
            description: "Master Input Slave Output pin",
            interfaceNames: ["RX"]});
    }

    if (mosiRequired) {
        spi.resources.push({
            name: "mosiPin",
            displayName: "MOSI Pin",
            description: "Master Output Slave Input pin",
            interfaceNames: ["TX"]});
    }

    /* add CS pin if one of the four pin modes is selected */
    if (inst.mode != "Three Pin") {
        spi.resources.push({
                name: "ssPin",
                displayName: "SS Pin",
                description: "Slave Select / Chip Select",
                interfaceNames: ["FSS"]
            });
    }

    if (rxRequired) {
        spi.resources.push({
            name: "dmaRxChannel",
            displayName: "DMA RX Channel",
            description: "DMA channel used to receive data",
            interfaceNames: ["DMA_RX"]});
    }

    if (txRequired) {
        spi.resources.push({
            name: "dmaTxChannel",
            displayName: "DMA TX Channel",
            description: "DMA channel used to send data",
            interfaceNames: ["DMA_TX"]});
    }

    spi.signalTypes = {
        sclkPin: ["SPI_SCLK"],
        mosiPin: ["SPI_MOSI"],
        misoPin: ["SPI_MISO"],
        ssPin:   ["DOUT", "SPI_SS"]
    };

    return ([spi]);
}

/*
 *  ======== moduleInstances ========
 *  returns PIN instances
 */
function moduleInstances(inst)
{
    let pinInstances = new Array();

    pinInstances.push(
        {
            name: "sclkPinInstance",
            displayName: "SPI Clock PIN Configuration While Pin is Not In Use",
            moduleName: "/ti/drivers/PIN",
            collapsed: true,
            args: { parentMod: "/ti/drivers/SPI",
                    parentInterfaceName: "spi",
                    parentSignalName: "sclkPin",
                    parentSignalDisplayName: "SCLK",
                    mode: "Output",
                    outputState: "Low",
                    pull: "None" }
        }
    );

    pinInstances.push(
        {
            name: "misoPinInstance",
            displayName: "SPI MISO PIN Configuration While Pin is Not In Use",
            moduleName: "/ti/drivers/PIN",
            collapsed: true,
            args: { parentMod: "/ti/drivers/SPI",
                    parentInterfaceName: "spi",
                    parentSignalName: "misoPin",
                    parentSignalDisplayName: "MISO",
                    mode: "Input",
                    pull: "None" }
        }
    );

    pinInstances.push(
        {
            name: "mosiPinInstance",
            displayName: "SPI MOSI PIN Configuration While Pin is Not In Use",
            moduleName: "/ti/drivers/PIN",
            collapsed: true,
            args: { parentMod: "/ti/drivers/SPI",
                    parentInterfaceName: "spi",
                    parentSignalName: "mosiPin",
                    parentSignalDisplayName: "MOSI",
                    mode: "Output",
                    outputState: "Low",
                    pull: "None" }
        }
    );

    if (inst.mode != "Three Pin") {
        pinInstances.push(
            {
                name: "ssPinInstance",
                displayName: "SPI CS PIN Configuration While Pin is Not In Use",
                moduleName: "/ti/drivers/PIN",
                collapsed: true,
                args: { parentMod: "/ti/drivers/SPI",
                        parentInterfaceName: "spi",
                        parentSignalName: "ssPin",
                        parentSignalDisplayName: "SS",
                        mode: "Output",
                        outputState: inst.mode.match("high") ? "Low" : "High",
                        pull: "None" }
            }
        );
    }

    return (pinInstances);
}

/*
 *  ======== extend ========
 *  Extends a base exports object to include any device specifics
 *
 *  This function is invoked by the generic SPI module to
 *  allow us to augment/override as needed for the CC32XX
 */
function extend(base)
{
    /* display which driver implementation can be used */
    base = Common.addImplementationConfig(base, "SPI", null,
        [{name: "SPICC26X2DMA"}], null);

    /* merge and overwrite base module attributes */
    let result = Object.assign({}, base, devSpecific);

    /* concatenate device-specific configs */
    result.config = base.config.concat(devSpecific.config);

    /* duplex not currently supported */
    let index = result.config.findIndex(index => index.name === "duplex");
    if (index) {
        result.config[index].hidden = true;
        result.config[index].readOnly = true;
    }

    return (result);
}

/*
 *  ======== exports ========
 *  Export device-specific extensions to base exports
 */
exports = {
    /* required function, called by base SPI module */
    extend: extend
};
