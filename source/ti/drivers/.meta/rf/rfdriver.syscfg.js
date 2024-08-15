/*
 * Copyright (c) 2020, Texas Instruments Incorporated - http://www.ti.com
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
 *  ======== rfdriver.syscfg.js ========
 */

"use strict";

/* Common /ti/drivers utility functions */
const Common = system.getScript("/ti/drivers/Common.js");

/* Documentation for module config options */
const Docs = system.getScript("/ti/drivers/rf/RF.docs.js");

/* Pin symbol default prefix */
const SYM_PREFIX = "CONFIG_RF_";

/* HW supported for antenna switch code generation */
const hwSupported = [
    "SKY13317-373LF", // CC1352P
    "XMSSJJ3G0PA-054" // CC2652PSIP
];

/* Options for global event mask */
const globalEventOptions = [
    {
        name: "RF_GlobalEventRadioSetup",
        description: Docs.globalEvent.radioSetup.description
    },
    {
        name: "RF_GlobalEventRadioPowerDown",
        description: Docs.globalEvent.radioPowerDown.description
    },
    {
        name: "RF_GlobalEventInit",
        description: Docs.globalEvent.init.description
    },
    {
        name: "RF_GlobalEventCmdStart",
        description: Docs.globalEvent.cmdStart.description
    },
    {
        name: "RF_GlobalEventCmdStop",
        description: Docs.globalEvent.cmdStop.description
    },
    {
        name: "RF_GlobalEventCoexControl",
        description: Docs.globalEvent.coexControl.description
    }
];

/* Required RF events for specific features */
const requiredGlobalEvents = {
    antennaSwitch: [
        "RF_GlobalEventInit",
        "RF_GlobalEventRadioSetup",
        "RF_GlobalEventRadioPowerDown"
    ],
    coex: [
        "RF_GlobalEventInit",
        "RF_GlobalEventCmdStart",
        "RF_GlobalEventCmdStop",
        "RF_GlobalEventCoexControl"
    ]
};

/* Structure for coex config */
const coexConfig = {
    coExEnable: {
        bCoExEnable: 0,
        bUseREQUEST: 0,
        bUseGRANT: 0,
        bUsePRIORITY: 0,
        bRequestForChain: 0
    },
    coExTxRxIndication: 1,
    priorityIndicationTime: 20,
    overrideConfig: {
        bUseOverridePriority: 0,
        overridePriority: 0,
        bUseOverrideRequestForRx: 0,
        overrideRequestForRx: 0
    },
    cmdBleMasterSlaveConfig: {
        defaultPriority: 0,
        bAssertRequestForRx: 1,
        bIgnoreGrantInRx: 0,
        bKeepRequestIfNoGrant: 0
    },
    cmdBleAdvConfig: {
        defaultPriority: 0,
        bAssertRequestForRx: 1,
        bIgnoreGrantInRx: 0,
        bKeepRequestIfNoGrant: 0
    },
    cmdBleScanConfig: {
        defaultPriority: 0,
        bAssertRequestForRx: 1,
        bIgnoreGrantInRx: 0,
        bKeepRequestIfNoGrant: 0
    },
    cmdBleInitConfig: {
        defaultPriority: 0,
        bAssertRequestForRx: 1,
        bIgnoreGrantInRx: 0,
        bKeepRequestIfNoGrant: 0
    },
    cmdBleGenericRxConfig: {
        defaultPriority: 0,
        bAssertRequestForRx: 1,
        bIgnoreGrantInRx: 0,
        bKeepRequestIfNoGrant: 0
    },
    cmdBleTxTestConfig: {
        defaultPriority: 0,
        bAssertRequestForRx: 1,
        bIgnoreGrantInRx: 0,
        bKeepRequestIfNoGrant: 0
    }
};

/* Map option to enum name */
const priorityEnumLookup = {
    0: "RF_PriorityCoexLow",
    1: "RF_PriorityCoexHigh"
};
const requestEnumLookup = {
    0: "RF_RequestCoexNoAssertRx",
    1: "RF_RequestCoexAssertRx"
};

/* Structure for BLE specific config */
const coexConfigBle = {};

/* Array of level options */
const coexLevelOptions = [
    {
        name: 0,
        displayName: "Low"
    },
    {
        name: 1,
        displayName: "High"
    }
];

/* Define max number of antenna pins */
const N_MAX_ANTENNA_PINS = 10;

/* Construct configuration instance for interrupt priority */
const intPriority = Common.newIntPri()[0];
intPriority.description = Docs.intPriority.description;

/* Construct configuration instance for software interrupt priority */
const swiPriority = Common.newSwiPri();
swiPriority.description = Docs.swiPriority.description;

/* Configurables for the rfdriver module */
const config = [
    {
        name: "pinSelectionAntenna",
        displayName: Docs.pinSelectionAntenna.displayName,
        description: Docs.pinSelectionAntenna.description,
        longDescription: Docs.pinSelectionAntenna.longDescription,
        default: 0,
        options: Array.from(Array(N_MAX_ANTENNA_PINS+1), (x,i) => i).map(i => ({name:i})),
        hidden: false,
        onChange: onPinSelectionAntennaChanged
    },
    {
        name: "pinSelection",
        displayName: "RF Antenna Multiplexer Pin Selections",
        description: "This option is deprecated",
        longDescription: `
This option is deprecated and replaced by 'pinSelectionAntenna'.
`,
        minSelections: 0,
        hidden: true,
        default: [],
        options: getPinOptions(),
        deprecated: true,
        onChange: onPinSelectionChanged
    },
    {
        name: "coexEnable",
        displayName: Docs.coex.enable.displayName,
        description: Docs.coex.enable.description,
        longDescription: Docs.coex.enable.longDescription,
        default: false,
        onChange: onCoexEnableChanged
    },
    {
        name: "coexConfigGroup",
        displayName: Docs.coex.configGroup.displayName,
        collapsed: false,
        config: [
            {
                name: "coexMode",
                displayName: Docs.coex.mode.displayName,
                description: Docs.coex.mode.description,
                longDescription: Docs.coex.mode.longDescription,
                hidden: true,
                default: "coexMode3Wire",
                options: [
                    {
                        name: "coexMode3Wire",
                        displayName: Docs.coex.mode.threeWire.displayName,
                        description: Docs.coex.mode.threeWire.description
                    },
                    {
                        name: "coexMode1WireRequest",
                        displayName: Docs.coex.mode.oneWireRequest.displayName,
                        description: Docs.coex.mode.oneWireRequest.description
                    },
                    {
                        name: "coexMode1WireGrant",
                        displayName: Docs.coex.mode.oneWireGrant.displayName,
                        description: Docs.coex.mode.oneWireGrant.description
                    }
                ],
                onChange: onCoexModeChanged
            },
            {
                name: "coexPinRequestIdleLevel",
                displayName: Docs.coex.pinIdleLevel.displayName.replace("%S%", "REQUEST"),
                description: Docs.coex.pinIdleLevel.description.replace("%S%", "REQUEST"),
                longDescription: Docs.coex.pinIdleLevel.longDescription.replace("%S%", "REQUEST"),
                hidden: true,
                default: 0,
                options: coexLevelOptions
            },
            {
                name: "coexPinPriorityIdleLevel",
                displayName: Docs.coex.pinIdleLevel.displayName.replace("%S%", "PRIORITY"),
                description: Docs.coex.pinIdleLevel.description.replace("%S%", "PRIORITY"),
                longDescription: Docs.coex.pinIdleLevel.longDescription.replace("%S%", "PRIORITY"),
                hidden: true,
                default: 0,
                options: coexLevelOptions
            },
            {
                name: "coexPinPriorityIndicationTime",
                displayName: Docs.coex.priorityIndicationTime.displayName,
                description: Docs.coex.priorityIndicationTime.description,
                longDescription: Docs.coex.priorityIndicationTime.longDescription,
                hidden: true,
                default: 20,
                options: [10, 15, 20, 25, 30].map(it => ({name: it})),
                onChange: onCoexPriorityIndicationTimeChanged
            },
            {
                name: "coexPinGrantIdleLevel",
                displayName: Docs.coex.pinIdleLevel.displayName.replace("%S%", "GRANT"),
                description: Docs.coex.pinIdleLevel.description.replace("%S%", "GRANT"),
                longDescription: Docs.coex.pinIdleLevel.longDescription.replace("%S%", "GRANT"),
                hidden: true,
                default: 1,
                options: coexLevelOptions
            },
            {
                name: "coexUseCaseConfigGroup",
                displayName: Docs.coex.useCaseConfigGroup.displayName,
                collapsed: true,
                config: [
                    {
                        name: "bleIniGroup",
                        displayName: Docs.coex.useCaseConfigGroup.ini.displayName,
                        collapsed: false,
                        config: [
                            {
                                name: "bleIniDefaultPriority",
                                displayName: Docs.coex.defaultPriority.displayName,
                                description: Docs.coex.defaultPriority.description,
                                longDescription: Docs.coex.defaultPriority.longDescription,
                                hidden: true,
                                default: 0,
                                options: coexLevelOptions,
                                onChange: updateCoexConfig
                            },
                            {
                                name: "bleIniAssertRequestForRx",
                                displayName: Docs.coex.assertRequestForRx.displayName,
                                description: Docs.coex.assertRequestForRx.description,
                                longDescription: Docs.coex.assertRequestForRx.longDescription,
                                hidden: true,
                                default: true,
                                onChange: updateCoexConfig
                            }
                        ]
                    },
                    {
                        name: "bleConGroup",
                        displayName: Docs.coex.useCaseConfigGroup.con.displayName,
                        collapsed: false,
                        config: [
                            {
                                name: "bleConDefaultPriority",
                                displayName: Docs.coex.defaultPriority.displayName,
                                description: Docs.coex.defaultPriority.description,
                                longDescription: Docs.coex.defaultPriority.longDescription,
                                hidden: true,
                                default: 0,
                                options: coexLevelOptions,
                                onChange: updateCoexConfig
                            },
                            {
                                name: "bleConAssertRequestForRx",
                                displayName: Docs.coex.assertRequestForRx.displayName,
                                hidden: true,
                                default: true,
                                onChange: updateCoexConfig
                            }
                        ]
                    },
                    {
                        name: "bleBroGroup",
                        displayName: Docs.coex.useCaseConfigGroup.bro.displayName,
                        collapsed: false,
                        config: [
                            {
                                name: "bleBroDefaultPriority",
                                displayName: Docs.coex.defaultPriority.displayName,
                                description: Docs.coex.defaultPriority.description,
                                longDescription: Docs.coex.defaultPriority.longDescription,
                                hidden: true,
                                default: 0,
                                options: coexLevelOptions,
                                onChange: updateCoexConfig
                            },
                            {
                                name: "bleBroAssertRequestForRx",
                                displayName: Docs.coex.assertRequestForRx.displayName,
                                hidden: true,
                                default: true,
                                onChange: updateCoexConfig
                            }
                        ]
                    },
                    {
                        name: "bleObsGroup",
                        displayName: Docs.coex.useCaseConfigGroup.obs.displayName,
                        collapsed: false,
                        config: [
                            {
                                name: "bleObsDefaultPriority",
                                displayName: Docs.coex.defaultPriority.displayName,
                                description: Docs.coex.defaultPriority.description,
                                longDescription: Docs.coex.defaultPriority.longDescription,
                                hidden: true,
                                default: 0,
                                options: coexLevelOptions,
                                onChange: updateCoexConfig
                            },
                            {
                                name: "bleObsAssertRequestForRx",
                                displayName: Docs.coex.assertRequestForRx.displayName,
                                hidden: true,
                                default: true,
                                onChange: updateCoexConfig
                            }
                        ]
                    }
                ]
            }
        ]
    },
    intPriority,
    swiPriority,
    {
        name: "xoscNeeded",
        displayName: Docs.xoscNeeded.displayName,
        description: Docs.xoscNeeded.description,
        longDescription: Docs.xoscNeeded.longDescription,
        default: true
    },
    {
        name: "globalEventMask",
        displayName: Docs.globalEventMask.displayName,
        description: Docs.globalEventMask.description,
        longDescription: Docs.globalEventMask.longDescription,
        minSelections: 0,
        default: [],
        onChange: onGlobalEventMaskChanged,
        options: globalEventOptions
    },
    {
        name: "globalCallbackFunction",
        displayName: Docs.globalCallbackFunction.displayName,
        description: Docs.globalCallbackFunction.description,
        longDescription: Docs.globalCallbackFunction.longDescription,
        default: "NULL",
        readOnly: true
    },
    {
        name: "pinSymGroup",
        displayName: Docs.pinSymGroup.displayName,
        collapsed: false,
        config: getPinSymbolConfigurables()
    }
];

/*
 *******************************************************************************
 Status Functions
 *******************************************************************************
 */

/*
 *  ======== filterHardware ========
 *  Check component signals for compatibility with RF.
 *  
 *  @param component    - A hardware component
 *  @return             - Bool
 */
function filterHardware(component) {
    return (Common.typeMatches(component.type, ["RF"]));
}

/* 
 *  ======== isCustomDesign ========
 *  Check if device used is custom (i.e. not LaunchPad).
 *  
 *  @return - Bool
 */
function isCustomDesign() {
    const deviceData = system.deviceData;
    return !("board" in deviceData);
}

/*
 *  ======== isPhy ========
 *  Check if module is shared by (i.e. child of) a specific phy.
 *  
 *  @param inst - Module instance object
 *  @param phy  - "ble", "ieee_15_4" or "prop"
 *  @return     - Bool
 */
function isPhy(inst, phy) {
    const status = (inst.$sharedBy.find(obj => obj.$module.$name.includes(`settings/${phy}`))) ? true : false;
    return status;
}

/*
 *  ======== isConfigDefaultValue ========
 *  Check if specified configuration option has its default value.
 *  
 *  @param inst     - Module instance object containing the config
 *  @param config   - Config option to get default value of
 *  @return         - Bool
 */
function isConfigDefaultValue(inst, config){
    return (inst[config] === getDefaultValue(inst, config));
}

/*
 *******************************************************************************
 Get Functions
 *******************************************************************************
 */

/*
 *  ======== getDefaultValue ========
 *  Get default value for specified configuration option.
 *  
 *  @param inst     - Module instance object containing the config
 *  @param config   - Config option to get default value of
 *  @return         - Default/initial value used for config
 */
function getDefaultValue(inst, config){
    return (config in inst) ? inst.$module.$configByName[config].default : undefined;
}

/*
 *  ======== getPinByDIO ========
 *  Get a pin description.
 *  
 *  @param   dio - Formatted "DIO_XX"
 *  @return      - Pin object 
 */
function getPinByDIO(dio) {
    const deviceData = system.deviceData;
    const ipin = _.findKey(deviceData.devicePins, pin => pin.designSignalName === dio);
    if (ipin === undefined) {
        throw Error("Pin not found: " + dio);
    }
    return deviceData.devicePins[ipin];
}

/*
 *  ======== getPinOptions ========
 *  Get a list of available DIO pins.
 * 
 *  @return - Array of pin options {name, displayName} 
 */
function getPinOptions() {
    const deviceData = system.deviceData;
    const dioPins = Object.values(deviceData.devicePins).filter(pin => pin.designSignalName.includes("DIO_"));
    
    const pinOptions = dioPins.map(pin => ({
            name: pin.designSignalName,
            /* Display as "DIO_X / Pin Y" */
            displayName: pin.designSignalName + " / Pin " + pin.ball
    }));

    return pinOptions;
}

/*
 *  ======== getPinSymbolConfigurables ========
 *  Get pin symbol configurables based on device pin options.
 *  
 *  @return - Array of pin symbol objects
 */
function getPinSymbolConfigurables() {
    const config = [];
    
    /*
    *  ======== NOT USED ========
    *  The following was used to map custom symbols to DIOs,
    *  by adding all available pin options as module configurables.
    *  Not in use, but kept for backwards compatibility.
    */
    const pinOptions = getPinOptions();
    _.each(pinOptions, (opt) => {
        const name = opt.name;
        const pinSymCfg = {
            name: name,
            displayName: name,
            default: SYM_PREFIX + name.replace("_", ""),
            hidden: true
        };
        config.push(pinSymCfg);
    });
    /* ==========================

    /* Add pin symbol config for antenna pins */
    for(let index = 0; index < N_MAX_ANTENNA_PINS; index++) {
        config.push({
            name: `rfAntennaPinSymbol${index}`,
            displayName: `RF Antenna Pin ${index}`,
            default: SYM_PREFIX + `ANTENNA_PIN_${index}`,
            hidden: true
        })
    }

    /* Add pin symbol config for coex pins*/
    const coexSignalNames = ["REQUEST", "PRIORITY", "GRANT"];
    coexSignalNames.forEach(signal => {
        const displayName = `RF Coex ${signal} Pin`;
        config.push({
            name: _.camelCase(displayName + " Symbol"),
            displayName: displayName,
            default: SYM_PREFIX + `COEX_${signal}`,
            hidden: true
        });
    });

    return config;
}

/*
 *  ======== getCoexPinInfo ========
 *  Get list with info on enabled coex signals.
 *  
 *  @param inst     - Module instance object containing the config
 *  @return         - Array of coex pin objects
 */
function getCoexPinInfo(inst) {
    const {coexEnable: enable, coexMode: mode} = inst;

    const request = {
        name: "rfCoexRequestPin",
        displayName: "RF Coex REQUEST Pin",
        signal: "REQUEST",
        pinArgs: {
            $name: inst["rfCoexRequestPinSymbol"]
        }
    };

    const priority = {
        name: "rfCoexPriorityPin",
        displayName: "RF Coex PRIORITY Pin",
        signal: "PRIORITY",
        pinArgs: {
            $name: inst["rfCoexPriorityPinSymbol"]
        }
    };

    const grant = {
        name: "rfCoexGrantPin",
        displayName: "RF Coex GRANT Pin",
        signal: "GRANT",
        pinArgs: {
            $name: inst["rfCoexGrantPinSymbol"]
        }
    };
    
    const coexPinInfo = [];
    if(enable){
        switch(mode) {
            case "coexMode3Wire":
                coexPinInfo.push(request, priority, grant);
                break;
            case "coexMode1WireRequest":
                coexPinInfo.push(request);
                break;
            case "coexMode1WireGrant":
                coexPinInfo.push(grant);
                break;
        }
    }
    return coexPinInfo;
}

/*
 *******************************************************************************
 OnChange functions
 *******************************************************************************
 */

function onHardwareChanged(inst, ui) {
    const hardware = inst.$hardware;
    const hasHardware = hardware !== null;

    /* Update pin symbol names, if any signals */
    const hwSignals = hasHardware ? Object.keys(hardware.signals) : [];
    hwSignals.forEach((signal, index) => {
        inst[`rfAntennaPinSymbol${index}`] = SYM_PREFIX + hardware.signals[signal].name;
    });

    /* Update antenna pin selection to number of HW signals */
    updateConfigValue(inst, ui, "pinSelectionAntenna", hwSignals.length);
    ui.pinSelectionAntenna.readOnly = hasHardware;
}

/*
 *  ======== onPinSelectionChanged ========
 *  Called when config pinSelection changes.
 * 
 *  @param inst - Module instance object containing config that changed
 *  @param ui   - User Interface state object
 */
function onPinSelectionChanged(inst, ui) {
    /* Deprecated.
     * Write length of pinSelection list to
     * pinSelectionAntenna to support new selection approach.
     */
    const length = inst.pinSelection.length;
    const nPins = length > N_MAX_ANTENNA_PINS ? N_MAX_ANTENNA_PINS : length;
    updateConfigValue(inst, ui, "pinSelectionAntenna", nPins);
}

/*
 *  ======== onPinSelectionAntennaChanged ========
 *  Called when config pinSelectionAntenna changes.
 * 
 *  @param inst - Module instance object containing config that changed
 *  @param ui   - User Interface state object
 */
function onPinSelectionAntennaChanged(inst, ui) {
    updatePinSymbols(inst, ui);

    /* Update global event mask */
    let events = [];
    if(inst.pinSelectionAntenna > 0) {
        /* Combine existing event mask with required events */
        events = [].concat(inst.globalEventMask, requiredGlobalEvents.antennaSwitch);
    }
    else {
        /* Strip out the required events from the event mask */
        events = inst.globalEventMask.filter(event => !requiredGlobalEvents.antennaSwitch.includes(event));
    }
    updateConfigValue(inst, ui, "globalEventMask", events);
}

/*
 *  ======== onGlobalEventMaskChanged ========
 *  Called when config globalEventMask changes
 * 
 *  @param inst - Module instance object containing config that changed
 *  @param ui   - User Interface state object
 */
function onGlobalEventMaskChanged(inst, ui) {
    const noEvents = (inst.globalEventMask.length === 0)
    if (noEvents) {
        ui.globalCallbackFunction.readOnly = true;
        resetDefaultValue(inst, "globalCallbackFunction");
    }
    else {
        ui.globalCallbackFunction.readOnly = false;
        if (isConfigDefaultValue(inst, "globalCallbackFunction")) {
            inst.globalCallbackFunction = "rfDriverCallback";
        }
    }
}

/*
 *  ======== onCoexEnableChanged ========
 *  Called when config coexEnable changes
 * 
 *  @param inst - Module instance object containing config that changed
 *  @param ui   - User Interface state object
 */
function onCoexEnableChanged(inst, ui) {
    const enabled = inst.coexEnable;
    const coexInstances = Object.keys(ui).filter(key => ((key.includes("coex") || key.includes("ble")) && key !== "coexEnable"));

    /* Show coex config instances according to coexEnable state */
    coexInstances.forEach(instance => ui[instance].hidden = !enabled);

    /* Reset config instances to default value if hidden */
    coexInstances.filter(instance => ui[instance].hidden).forEach(instance => resetDefaultValue(inst, instance));
    
    /* Update global event mask */
    let events = [];
    if(enabled) {
        /* Combine existing event mask with required events */
        events = [].concat(inst.globalEventMask, requiredGlobalEvents.coex);
    }
    else {
        /* Strip out the required events from the event mask */
        events = inst.globalEventMask.filter(event => !requiredGlobalEvents.coex.includes(event));
    }
    updateConfigValue(inst, ui, "globalEventMask", events);

    /* Update pin symbols */
    updatePinSymbols(inst, ui);

    /* Update coex config structure */
    updateCoexConfig(inst);
}

/*
 *  ======== onCoexModeChanged ========
 *  Called when config coexMode changes
 * 
 *  @param inst - Module instance object containing config that changed
 *  @param ui   - User Interface state object
 */
function onCoexModeChanged(inst, ui) {
    const mode = inst.coexMode;
    const coexPinInstances = Object.keys(ui).filter(key => key.includes("coexPin"));

    /* Update pin config visibility according to mode */
    coexPinInstances.forEach(instance => {
        if(instance.includes("Request")) {
            ui[instance].hidden = (mode == "coexMode1WireGrant");
        }
        else if(instance.includes("Priority")) {
            ui[instance].hidden = (mode !== "coexMode3Wire");
        }
        else if(instance.includes("Grant")) {
            ui[instance].hidden = (mode == "coexMode1WireRequest");
        }
    });

    /* Update ble config visibility according to mode */
    const coexBleInstances = Object.keys(ui).filter(key => key.substring(0,3) === "ble");
    coexBleInstances.forEach(instance => {
        if(instance.includes("DefaultPriority")) {
            ui[instance].hidden = (mode !== "coexMode3Wire");
        }
        else if(instance.includes("AssertRequestForRx")) {
            ui[instance].hidden = (mode == "coexMode1WireGrant");
        }
    });

    /* Reset config instance to default value if hidden */
    const hiddenInstances = coexPinInstances.concat(coexBleInstances).filter(instance => ui[instance].hidden);
    hiddenInstances.forEach(instance => resetDefaultValue(inst, instance));

    /* Update pin symbols */
    updatePinSymbols(inst, ui);

    /* Update coex config structure */
    updateCoexConfig(inst);
}

/*
 *  ======== onCoexPriorityIndicationTimeChanged ========
 *  Called when config coexPinPriorityIndicationTime changes
 * 
 *  @param inst - Module instance object containing config that changed
 *  @param ui   - User Interface state object
 */
function onCoexPriorityIndicationTimeChanged(inst, ui) {
    /* Update coex config structure */
    updateCoexConfig(inst);
}

/*
 *******************************************************************************
 Update functions
 *******************************************************************************
 */

/*
 *  ======== resetDefaultValue ========
 *  Set specified configuration option to it's default value
 *  
 *  @param inst     - Module instance object containing the config
 *  @param config   - Config option to reset
 */
function resetDefaultValue(inst, config){
    if(config in inst) {
        inst[config] = inst.$module.$configByName[config].default;
    }
}

/*
 *  ======== updateConfigValue ========
 *  Update config value and trigger onChange member function.
 *  
 *  @param inst     - Module instance object containing the config
 *  @param ui       - UI state object
 *  @param config   - Config option part of module
 *  @param value    - New value for config
 */
function updateConfigValue(inst, ui, config, value){
    if(config in inst) {
        inst[config] = value;
        inst.$module.$configByName[config].onChange(inst, ui);
    }
}

/*
 *  ======== updatePinSymbols ========
 *  Update pin symbol visibility according to selected pins.
 * 
 *  @param inst - Module instance object containing config that changed
 *  @param ui   - User Interface state object
 */
function updatePinSymbols(inst, ui) {
    const pinSymbolInstances = Object.keys(inst).filter(key => key.match("PinSymbol"));

    /* Hide all pin symbol instances */
    pinSymbolInstances.forEach(symbol => ui[symbol].hidden = true);

    /* Show enabled antenna pins */
    const nPins = inst.pinSelectionAntenna;
    const hasHardware = (inst.$hardware !== null);
    for(let i = 0; i < nPins; i++) {
        const symInst = `rfAntennaPinSymbol${i}`
        ui[symInst].hidden = false;
        ui[symInst].readOnly = hasHardware;
    }
    
    /* Show enabled coex pins */
    const coexPinInfo = getCoexPinInfo(inst);
    coexPinInfo.forEach(pin => {
        const symInst = `${pin.name}Symbol`
        ui[symInst].hidden = false;
        ui[symInst].readOnly = true;
    });

    /* Reset pin to default value if hidden */
    pinSymbolInstances.filter(instance => ui[instance].hidden).forEach(instance => {
        resetDefaultValue(inst, instance);
        ui[instance].readOnly = false;
    });
}

/*
*  ======== updateCoexConfig ========
*  Update coex configuration stored in coexConfig structure.
*
*  @param inst - Module instance object containing config that changed
*/
function updateCoexConfig(inst) {
    const {coexEnable: enable, coexMode: mode, coexPinPriorityIndicationTime: priIndicationTime} = inst;

    /* Coex state info */
    coexConfig.coExEnable = {
        bCoExEnable: Number(enable),
        bUseREQUEST: Number(enable && (mode === "coexMode3Wire" || mode === "coexMode1WireRequest")),
        bUseGRANT: Number(enable && (mode === "coexMode3Wire" || mode === "coexMode1WireGrant")),
        bUsePRIORITY: Number(enable && mode === "coexMode3Wire")
    };
    coexConfig.priorityIndicationTime = priIndicationTime;

    /* BLE Use Case Config info */
    coexConfigBle.bleInitiator = {
        defaultPriority: priorityEnumLookup[inst.bleIniDefaultPriority],
        assertRequestForRx: requestEnumLookup[Number(inst.bleIniAssertRequestForRx)]
    };
    coexConfigBle.bleConnected = {
        defaultPriority: priorityEnumLookup[inst.bleConDefaultPriority],
        assertRequestForRx: requestEnumLookup[Number(inst.bleConAssertRequestForRx)]
    };
    coexConfigBle.bleBroadcaster = {
        defaultPriority: priorityEnumLookup[inst.bleBroDefaultPriority],
        assertRequestForRx: requestEnumLookup[Number(inst.bleBroAssertRequestForRx)]
    };
    coexConfigBle.bleObserver = {
        defaultPriority: priorityEnumLookup[inst.bleObsDefaultPriority],
        assertRequestForRx: requestEnumLookup[Number(inst.bleObsAssertRequestForRx)]
    };
}

/*
 *******************************************************************************
 Module Dependencies
 *******************************************************************************
 * When change occurs within module, the default module functions (if declared)
 * will be triggered when a configurable changes state:
 *  - pinmuxRequirements(...)
 *  - moduleInstances(...)
 *  - sharedModuleInstances(...)
 *  - modules(...)
 */

/*!
 *  ======== pinmuxRequirements ========
 *  Return peripheral pin requirements as a function of config.
 * 
 *  Called when a configuration changes in the module.
 *
 *  @param inst - Module instance containing the config that changed
 *  @return     - Array of pin requirements
 */
function pinmuxRequirements(inst) {
    const rfArray = [];

    /* Requirements for antenna pins */
    const hardware = inst.$hardware;
    const hasHardware = hardware !== null;
    const dioHardwarePins = hasHardware ? Object.values(hardware.signals).map(s => s.devicePin.description) : [];
    
    const nPins = inst.pinSelectionAntenna;
    for(let i = 0; i < nPins; i++) {
        const pinReq = {
            name: "rfAntennaPin" + i,
            displayName: "RF Antenna Pin " + i,
            hidden: false,
            interfaceName: "GPIO",
            signalTypes: ["RF"]
        };
        if(hasHardware) {
            pinReq.filter = (pin) => pin.designSignalName === dioHardwarePins[i];
        }
        rfArray.push(pinReq);
    }

    /* Requirements for coex pins */
    const coexPinInfo = getCoexPinInfo(inst);
    coexPinInfo.forEach(pin => {
        rfArray.push({
            name: pin.name,
            displayName: pin.displayName,
            interfaceName: "GPIO",
            signalTypes: ["RF"]
        });
    });

    return rfArray;
}

/*
 *  ======== moduleInstances ========
 *  Determines what modules are added as non-static submodules.
 * 
 *  Called when a configuration changes in the module.
 *
 *  @param inst - Module instance containing the config that changed
 *  @return     - Array of PIN instances
 */
function moduleInstances(inst) {
    const dependencyModules = [];

    /* Add PIN instances for antenna pins */
    const nPins = inst.pinSelectionAntenna;
    for(let i = 0; i < nPins; i++) {
        const pinInstance = {
            name: `rfAntennaPin${i}Instance`,
            displayName: `PIN Configuration While Antenna Pin ${i} Is Not In Use`,
            moduleName: "/ti/drivers/PIN",
            readOnly: false,
            collapsed: true,
            args: {
                $name: inst[`rfAntennaPinSymbol${i}`],
                parentMod: "/ti/drivers/RF",
                parentSignalName: `rfAntennaPin${i}`,
                parentSignalDisplayName: `RF Antenna Pin ${i}`,
            }
        };
        dependencyModules.push(pinInstance);
    }

    /* Add PIN instances for coex pins */
    const coexPinInfo = getCoexPinInfo(inst);
    coexPinInfo.forEach(pin => {
        const pinInstance = {
            name: pin.name + "Instance",
            displayName: `PIN Configuration While Coex ${pin.signal} Pin Is Not In Use`,
            moduleName: "/ti/drivers/PIN",
            readOnly: false,
            collapsed: true,
            args: {
                parentMod: "/ti/drivers/RF",
                parentSignalName: pin.name,
                parentSignalDisplayName: pin.displayName,
            }
        };
        pinInstance.args = {...pinInstance.args, ...pin.pinArgs};
        dependencyModules.push(pinInstance);
    })

    return dependencyModules;
}

/*
 *  ======== sharedModuleInstances ========
 *  Determines what modules are added as shared static sub-modules.
 * 
 *  Called when a configuration changes in the module.
 *
 *  @param inst - Module instance containing the config that changed
 *  @return     - Array containing dependency modules
 */
function sharedModuleInstances(inst){
    return [];
}

/*
 *  ======== modules ========
 *  Determines what modules are added as static sub-modules.
 *
 *  Called when a configuration changes in the module.
 * 
 *  @param inst - Module instance containing the config that changed
 *  @return     - Array containing static dependency modules
 */
function modules(inst) {
    const dependencies = ["Board", "Power", "Temperature"];

    return Common.autoForceModules(dependencies)();
}

/*
 *******************************************************************************
 GenLibs
 *******************************************************************************
 */

/*!
 *  ======== getLibs ========
 *  Return link option argument for GenLibs.
 *
 *  @param mod  - Module object
 *  @return     - Link option object
 */
function getLibs(mod) {
    /* Toolchain specific information */
    const GenLibs = system.getScript("/ti/utils/build/GenLibs");
    const isa = GenLibs.getDeviceIsa();
    const toolchain = GenLibs.getToolchainDir();
    const deviceId = system.deviceData.deviceId;

    /* Determine lib name */
    const prefix = "rf_multiMode_"
    const deviceFamily = !!(deviceId.match(/CC26.2/i)) ? "cc26x2" : "cc13x2";
    const suffix = Common.getLibSuffix(isa, toolchain);
    
    /* Create a GenLibs input argument */
    return {
        name: mod.$name,
        libs: [
            `ti/drivers/rf/lib/${prefix}${deviceFamily}${suffix}`
        ],
        deps: [
            "/ti/drivers"
        ]
    };
}

/*
 *******************************************************************************
 Module Validation
 *******************************************************************************
 */

/*!
 *  ======== validate ========
 *  Validate rfdriver module configuration.
 * 
 *  @param inst         - Module instance containing the config that changed
 *  @param validation   - Object to hold detected validation issues
 */
function validate(inst, validation) {
    const {$hardware: hardware, coexEnable: coexEnable, globalCallbackFunction: cbFxn, pinSelectionAntenna: nAntennaPins} = inst;
    
    if(coexEnable) {
        /* Coexistence feature is dependent on BLE */
        if(!isPhy(inst, "ble")){
            Common.logError(validation, inst, "coexEnable",
            "'RF Coexistence' is only supported with BLE PHY.");
        }
    }

    /* Check that globalCallbackFunction is a C identifier */
    if (!Common.isCName(cbFxn)) {
        Common.logError(validation, inst, "globalCallbackFunction",
        "'" + cbFxn + "' is not a valid a C identifier");
    }

    if (
        (cbFxn !== "NULL") && 
        (nAntennaPins > 0) && 
        ((hardware === null) || (!hwSupported.includes(hardware.name)))
    ) {
        /* Notify user that the antenna switching callback function must be implemented */
        Common.logInfo(validation, inst, "globalCallbackFunction",
        `Please see function '${cbFxn}AntennaSwitching' in 'ti_drivers_config.c'. The antenna switching functionality must be implemented by the user.`);
    }

    /* Verify that there are no pin name conflicts */
    const pinSymbolInstances = Object.keys(inst).filter(key => key.match("PinSymbol"));
    const pinNames = [];
    pinSymbolInstances.forEach(instance => {
        if(pinNames.includes(inst[instance])) {
            Common.logError(validation, inst, instance, "Conflicting PIN symbol names");
        }
        else {
            pinNames.push(inst[instance]);
        }
    });

    /* Antenna switch is required for PSIP board*/
    const board = isCustomDesign() ? null : system.deviceData.board;
    if ((board !== null) && (board.name.match(/PSIP/i)) && (hardware === null)) {
        Common.logError(validation, inst, "$hardware", `The board '${board.displayName}' requires the 'RF Antenna Switch' to be selected.`);
    }
}

/*
 *******************************************************************************
 Module Export
 *******************************************************************************
 */

 /*
 *  ======== rfdriverModule ========
 *  Define the RF Driver module properties and methods.
 */
const rfdriverModule = {
    moduleStatic: {
        config,
        pinmuxRequirements,
        moduleInstances,
        sharedModuleInstances,
        modules,
        validate,
        filterHardware,
        onHardwareChanged
    },
    isCustomDesign: isCustomDesign,
    getCoexConfig: function() {
        return coexConfig;
    },
    getCoexConfigBle: function() {
        return coexConfigBle;
    },

    /* override device-specific templates */
    templates: {
        /* contribute libraries to linker command file */
        "/ti/utils/build/GenLibs.cmd.xdt":
            {modName: "/ti/drivers/RF", getLibs},

        boardc: "/ti/drivers/rf/RF.Board.c.xdt",
        board_initc: "/ti/drivers/rf/RF.Board_init.c.xdt",
        boardh: "/ti/drivers/rf/RF.Board.h.xdt"
    }
}

/*
 *  ======== extend ========
 *  Extends a base exports object to include any device specifics
 *
 *  This function is invoked by the generic RF module to
 *  allow us to augment/override as needed for the CC26XX
 */
function extend(base) {
    /* merge and overwrite base module attributes */
    return({...base, ...rfdriverModule});
}

/*
 *  ======== exports ========
 *  Export device-specific extensions to base exports
 */
exports = {
    /* required function, called by generic RF module */
    extend: extend
};
