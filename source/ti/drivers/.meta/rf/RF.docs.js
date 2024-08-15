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
 *  ======== RF.docs.js ========
 */

/*
 *******************************************************************************
 Base Module
 *******************************************************************************
 */

const base = {
    displayName: "RF",
    description: "Radio Frequency (RF) Driver",
    longDescription: `
The [__Radio Frequency (RF) driver__][1] provides access to the radio core on
the CC13XX and CC26XX device family. It offers a high-level interface for
command execution and to the radio timer (RAT). The RF driver ensures the
lowest possible power consumption by providing automatic power management
that is fully transparent for the application.

For a detailed explanation of the RF core, please refer to the
[__Technical Reference Manual__][2] or the [__Proprietary RF User Guide__][3].

[1]: /rflib/rflib_api_documentation.html "Radio Software Bundle (rflib) API Documentation"
[2]: /proprietary-rf/technical-reference-manual.html
[3]: /proprietary-rf/proprietary-rf-users-guide.html
`
}

/*
 *******************************************************************************
 RF Driver Module
 *******************************************************************************
 */

const pinSelectionAntenna = {
    displayName: "No. of Antenna Switch Control Pins",
    description: "Number of pins to use for antenna switching",
    longDescription: `
Option to select number of pins to use for custom antenna switching, 
if no predefined hardware module is selected. 
`
};

const coex = {
    enable: {
        displayName: "RF Coexistence",
        description: "Enable RF coexistence feature",
        longDescription: `
Enable the use of external pin signaling to communicate with another RF capable
device, for the devices to coexist in the same frequency band.
The coexistence (coex) feature is a wired signal interface between a BLE device
and a Wi-Fi device:

\`\`\`
Coex slave              Coex master
+---------+             +---------+
|         |   Signals   |         |
|   BLE   | <----/----> |  Wi-Fi  | 
|         |      n      |         |
+---------+             +---------+
\`\`\`

Resources:
* __Coexistence__ chapter of the [TI BLE5-Stack User's Guide][UG]
* SimpleLink Academy [BLE Coexistence Configuration][SLA]

[UG]: http://dev.ti.com/tirex/explore/node?node=AOimuSWjap.4RuDbcp7OqA__pTTHBmu__LATEST
[SLA]: http://dev.ti.com/tirex/explore/node?node=AN3jFkz8RvbfdF-eMjQx0A__pTTHBmu__LATEST
`
    },
    configGroup: {
        displayName: "RF Coexistence Configuration"
    },
    mode: {
        displayName: "Coex Mode",
        description: "Select coex mode of operation",
        longDescription: `
The coex interface provides a set of signals based on the supported
Packet Traffic Arbitration (PTA) approach:

* REQUEST: Output signal, indicating a request to perform RF activity.
* PRIORITY: Output signal, time-shared between indicating (1) request priority and (2) type of RF activity.
* GRANT: Input signal, indicating the response to the request to perform RF activity.

The available combinations of these signals are defined as _coex modes_.
This option is used to match the interface of the coex master the device
is coexisting with. The following signals are enabled for the different modes:

|Coex Mode       | REQUEST | PRIORITY | GRANT |
|----------------|:-------:|:--------:|:-----:|
| 3-Wire         | x       | x        | x     |
| 1-Wire REQUEST | x       |          |       |
| 1-Wire GRANT   |         |          | x     |
`,
        threeWire: {
            displayName: "3-Wire",
            description: "Use pins for REQUEST, PRIORITY and GRANT signals"
        },
        oneWireRequest: {
            displayName: "1-Wire REQUEST",
            description: "Use pin for REQUEST signal"
        },
        oneWireGrant: {
            displayName: "1-Wire GRANT",
            description: "Use pin for GRANT signal"
        }
    },
    priorityIndicationTime: {
        displayName: "Priority Indication Time",
        description: "Duration in us for time-shared PRIORITY signal to indicate priority",
        longDescription: `
The PRIORITY signal is time-shared between indicating (1) request priority and (2) type of
RF activity. The __Priority Indication Time__ option specifies for how long in microseconds
(from when the REQUEST signal is asserted) the PRIORITY signal indicates (1) request priority.
After the specified time, the REQUEST signal indicates (2) type of RF activity.
`
    },
    pinIdleLevel: {
        displayName: "%S% Signal Idle Level",
        description: "Pin value when %S% signal is idle",
        longDescription: `
The signal idle level is the opposite of the signal active level. When the
%S% signal is configured with idle level _low_, the signal is active _high_.
This option is used to specify how the signal level should be interpreted.

| Idle level | Active level|
|:----------:|:-----------:|
| Low        | High        |
| High       | Low         |
`
    },
    defaultPriority: {
        displayName: "Default Priority",
        description: "Priority level used when time-shared PRIORITY signal indicates request priority",
        longDescription: `
This option sets the priority level used by the PRIORITY signal when it
indicates the priority of the coex request. The duration of this priority
level is defined by the __Priority Indication Time__ option. See the
__Coex Mode__ description for more information on what this signal is used for.
`
    },
    assertRequestForRx: {
        displayName: "Assert REQUEST Signal For RX",
        description: "Specify if REQUEST signal is asserted for RX commands",
        longDescription: `
If this option is _disabled_, the BLE device is configured to not assert the
REQUEST signal (and subsequently disregard any of the other coex signals)
when the scheduled RF activity is an RX command.
`
    },
    useCaseConfigGroup: {
        displayName: "BLE Use Case Configuration",
        ini: {
            displayName: "Connection Establishment"
        },
        con: {
            displayName: "Connected"
        },
        bro: {
            displayName: "Broadcaster"
        },
        obs: {
            displayName: "Observer"
        }
    }
};

const intPriority = {
    description: "RF peripheral hardware interrupt priority"
};

const swiPriority = {
    description: "RF driver software interrupt priority"  
};

const xoscNeeded = {
    displayName: "XOSC Needed",
    description: `
Specify if the High Frequency Crystal Oscillator
(XOSC-HF) shall always be started by the Power driver
`,
    longDescription: `
When __true__, the power driver always starts the XOSC-HF. When __false__, the RF
driver will request the XOSC-HF if needed.
`
};

const globalEventMask = {
    displayName: "Global Event Mask",
    description: "Sets global RF driver events",
    longDescription: `
This specifies a mask of global events which the __Global Callback Function__
is invoked upon.
`
};

const globalEvent = {
    radioSetup: {
        description: `
Global event triggered when the RF core
is being reconfigured through a setup.
`
    },
    radioPowerDown: {
        description: `
Global event triggered when the RF core
is being powered down.
`
    },
    init: {
        description: `
Global event triggered when the RF driver
is called for the first time.
`
    },
    cmdStart: {
        description: `
Global event triggered when the RF driver
dispatches a command chain to the RF core.
`
    },
    cmdStop: {
        description: `
Global event triggered when the RF driver
handles a command termination event.
`
    },
    coexControl: {
        description: `
Global event triggered when change to coex
configuration is requested.
`
    }
};

const globalCallbackFunction = {
    displayName: "Global Callback Function",
    description: "Function triggered by global RF driver events",
    longDescription: `
The RF driver serves additional global, client independent events by invoking
the *RF_globalCallbackFunction*, according to the __Global Event Mask__. This
callback function will invoke the __Global Callback Function__, named by the
user. The __Global Callback Function__ is declared _weak_ in the auto generated
file for the application to override it with a new definition, and handle the
triggering events as necessary. By setting the __Global Callback Function__ to
__NULL__, the default callback (if it exists) will be registered.

Global events triggering this callback can be configured through the
__Global Event Mask__ configuration. When enabled, please see the definition of
__Global Callback Function__ in 'ti_drivers_config.c' for examples/suggestions
on how to handle the selected events.

Some RF features, like antenna switching and coexistence, will provide additional
event handling. These callback functions are also declared _weak_ and are named
according to the selected __Global Callback Function__ name, with the feature as
postfix, i.e. \`rfDriverCallbackCoex(...)\` for the coexistence feature.
`
};

const pinSymGroup = {
    displayName: "RF Pin Symbols"
};

exports = {
    base,
    pinSelectionAntenna,
    coex,
    intPriority,
    swiPriority,
    xoscNeeded,
    globalEventMask,
    globalEvent,
    globalCallbackFunction,
    pinSymGroup
};
