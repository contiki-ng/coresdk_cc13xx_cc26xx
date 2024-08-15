# CC13xx/CC26xx SimpleLink Core SDK 5.40.02.00

## Source

The CoreSDK can be downloaded as part of the "SimpleLink™ CC13x2_26x2 SDK". While the SimpleLink™ SDK is released under a proprietary license, the Core SDK is released untder BSD 3-Clause license.

* Release notes of the SimpleLink™ CC13x2_26x2 SDK: https://software-dl.ti.com/simplelink/esd/simplelink_cc13x2_26x2_sdk/4.40.04.04/exports/docs/Documentation_Overview.html
* Release notes of the CC13xx/CC26xx SimpleLink Core SDK: https://software-dl.ti.com/simplelink/esd/simplelink_cc13x2_26x2_sdk/4.40.04.04/exports/docs/simplelink_mcu_sdk/release_notes_coresdk_cc13xx_cc26xx_5_40_02_00.html
* License: https://software-dl.ti.com/simplelink/esd/simplelink_cc13x2_26x2_sdk/4.40.04.04/exports/manifest_simplelink_cc13x2_26x2_sdk_4_40_04_04.html


# Extracting from SimpleLink™ CC13x2_26x2 SDK

The license manifest contains a list of all components of the SimpleLink™ CC13x2_26x2 SDK. Each files extracted from the SimpleLink™ CC13x2_26x2 SDK must be carefully compared with the license manifest to ensure that the file is part of the SimpleLink Core SDK and it is BSD licensed.

The following files are required for Contiki-NG:

* kernel/nortos
    * License: the whole folder "kernel" is part of the Core SDK (BSD-3-Clause), except some files in kernel/freertos and kernel/tirtos. So kernel/nortos is all BSD-3-Clause
    * The subfolders lib/ccs lib/gcc lib/iar and lib/ticlang are not used and can be deleted (saves ~50 MB)

* source/ti/devices
    * License: the folder is mostly part of Core SDK (BSD-3-Clause). source/ti/devices/cc13x2_cc26x2/ contains Radio Firmware (also BSD-3-Clause). source/ti/devices/radioconfig/.meta contains Radio Config (also BSD-3-Clause)
    * The subfolder can be deleted

* source/ti/drivers
    * License: the folder is mostly part of Core SDK (BSD-3-Clause). Some files belong to other components but all of these are BSD-3-Clause
    * The subfolders lib/ccs lib/gcc lib/iar and lib/ticlang are not used and can be deleted (saves ~50 MB)

* source/ti/posix
    * License: the folder is part of Core SDK (BSD-3-Clause)


