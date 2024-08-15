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
 *  ======== PWMMSP432E4.js ========
 */

"use strict";

/* get Common /ti/drivers utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

/*
 *  ======== devSpecific ========
 *  Device-specific extensions to be added to base PWM configuration
 */
let devSpecific = {
    config: [
        {
            name: "pwmGenerationMode",
            displayName: "PWM Generation Mode",
            description: "Specifies PWM count mode.",
            longDescription:`
A PWM generator can count in two different modes: count down mode or count
up/down mode.  In count down mode, it counts from a value down to zero,
and then resets to the preset value, producing left-aligned PWM signals
(that is, the rising edge of the two PWM signals produced by the generator
occur at the same time).  In count up/down mode, it counts up from zero
to the preset value, counts back down to zero, and then repeats the process,
producing center-aligned PWM signals (that is, the middle of the high/low
period of the PWM signals produced by the generator occurs at the same time).
`,
            default: "Down",
            options: [
                { name: "Down" },
                { name: "Up Down" }
            ]
        },
        {
            name: "synchronizationMode",
            displayName: "Synchronization Mode",
            description: "Specifies PWM synchronization mode.",
            longDescription: `
When the PWM generator parameters (period and pulse width) are modified, their
effect on the output PWM signals can be delayed.  In synchronous mode, the
parameter updates are not applied until a synchronization event occurs. This
mode allows multiple parameters to be modified and take effect simultaneously,
instead of one at a time. Additionally, parameters to multiple PWM generators
in synchronous mode can be updated simultaneously, allowing them to be treated
as if they were a unified generator. In non-synchronous mode, the parameter
updates are not delayed until a synchronization event. In either mode, the
parameter updates only occur when the counter is at zero to help prevent oddly
formed PWM signals during the update (that is, a PWM pulse that is too short
or too long).
`,
            default: "Immediate Updates",
            options: [
                { name: "Immediate Updates" },
                { name: "Local Synchronous Updates" },
                { name: "Global Synchronous Updates" }
            ]
        },
        {
            name: "debugMode",
            displayName: "Debug Mode",
            description: "Specifies PWM debug mode.",
            longDescription:`
The PWM generator can either pause or continue running when the processor is
stopped via the debugger. If configured to pause, it continues to count until
it reaches zero, at which point it pauses until the processor is restarted. If
configured to continue running, it keeps counting as if nothing had happened.
`,
            default: "Debug Run",
            options: [
                { name: "Debug Run" },
                { name: "Debug Stop" }
            ]
        }
    ],

    /* override generic requirements with  device-specific reqs (if any) */
    pinmuxRequirements: pinmuxRequirements,

    filterHardware: filterHardware,

    /* override device-specific templates */
    templates: {
        boardc: "/ti/drivers/pwm/PWMMSP432E4.Board.c.xdt",
        boardh: "/ti/drivers/pwm/PWMTimer.Board.h.xdt"
    },

    _getPinResources: _getPinResources
};

/*
 *  ======== _getPinResources ========
 */
function _getPinResources(inst)
{
    let pin;

    if (inst.pwm) {
        pin = inst.pwm.pwmPin.$solution.devicePinName;

        if (inst.$hardware && inst.$hardware.displayName) {
            pin += ", " + inst.$hardware.displayName;
        }
    }

    return (pin);
}

/*
 *  ======== pinmuxRequirements ========
 *  Returns peripheral pin requirements of the specified instance
 *
 *  param inst    - a fully configured PWM instance
 *
 *  returns req[] - an array of pin requirements needed by inst
 */
function pinmuxRequirements(inst)
{
    let pwm = {
        name: "pwm",
        displayName: "PWM Peripheral",
        interfaceName: "PWM",
        canShareWith: "PWM",
        resources: [
            {
                name: "pwmPin",
                displayName: "PWM Output Pin",
                interfaceNames: [
                    "PWM0", "PWM1", "PWM2", "PWM3", "PWM4", "PWM5",
                    "PWM6", "PWM7"
                ]
            }
        ],
        signalTypes: { pwmPin: ["PWM"] }
    };

    return ([pwm]);
}

/*
 *  ========= filterHardware ========
 *  Check 'component' signals for compatibility with PWM
 *
 *  param component - hardware object describing signals and
 *                    resources they're attached to
 *  returns Boolean indicating whether or not to allow the component to
 *           be assigned to an instance's $hardware config
 */
function filterHardware(component)
{
    for (let sig in component.signals) {
        let type = component.signals[sig].type;
        if (Common.typeMatches(type, ["PWM"])) {
            let devicePin = component.signals[sig].devicePin;
            if (devicePin) {
                if (isValidPwmPin(devicePin.mux.muxSetting)) {
                    return (true);
                }
            }
        }
    }

    return (false);
}

/*
 *  ======== isValidPwmPin ========
 *  Verify pin can support PWM
 */
function isValidPwmPin(muxSettings)
{
    for (let i in muxSettings) {
        if (muxSettings[i].peripheralPin.name.match("PWM")) {
            return (true);
        }
    }

    return (false);
}

/*
 *  ======== extend ========
 *  Extends a base exports object to include any device specifics
 *
 *  This function is invoked by the generic PWM module to
 *  allow us to augment/override as needed for the MSP432E4.
 */
function extend(base)
{
    /* display which driver implementation can be used */
    base = Common.addImplementationConfig(base, "PWM", null,
        [{name: "PWMMSP432E4"}], null);

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
    /* required function, called by base PWM module */
    extend: extend
};
