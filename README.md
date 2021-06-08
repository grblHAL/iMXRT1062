## iMXRT1062 Driver

A grblHAL driver for the NXP iMXRT1062 processor on a [Teensy 4.x board](https://www.pjrc.com/store/teensy40.html).

This driver compiles and uploads from the Arduino IDE and is partially dependent on the Arduino framework. [Teensyduino](https://www.pjrc.com/teensy/td_download.html) is required and must be added to the Arduino IDE.

See the Wiki-page for [compiling grblHAL](https://github.com/grblHAL/core/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

Available driver options can be found [here](main/my_machine.h).

---


#### Networking plugin

The networking plugin is for Teensy 4.1 and needs the [teensy41_ethernet lwIP library](https://github.com/ddrown/teensy41_ethernet) forked by ddrown.

Telnet and websocket protocols are currently supported, http is on the long term roadmap.

#### SD card plugin

The SD card plugin needs the [uSDFS library](https://github.com/WMXZ-EU/uSDFS) by WMXZ-EU.

Important: edit the [utility/sd_config.h](https://github.com/WMXZ-EU/uSDFS/blob/master/src/utility/sd_config.h) file and change

`#define USE_MSC 1	// will be used in sd_msc.cpp`

to

`#define USE_MSC 0	// will be used in sd_msc.cpp`

or add the MSC library as well \(not needed\). 2021-06-08: This is now changed in the latest version.

**NOTE:**

If enabling ftp transfer to the SD card then [utility/sd_sdhc.c](https://github.com/WMXZ-EU/uSDFS/blob/master/src/utility/sd_sdhc.c) has to be replaced with [this patched](patches/sd_sdhc.zip) version \(zip download\).  
I submitted a PR for this but it was rejected with no explanation, this is why I have added it here. The maintainer has made a similar change but that does not fix the underlying issue, and it may even crash the controller.  
In addition to this [ffconf.h](https://github.com/WMXZ-EU/uSDFS/blob/master/src/ffconf.h) has to be edited, `#define FF_FS_RPATH` value has to be changed to 2 \(from 1\) or you will get a compiler error.

---

Download the libraries above as zip files and add to your Arduino installation with _Sketch > Include Library > Add .ZIP Library..._

---
#### Board maps:

|                                                                              |N_AXIS|Ganged&nbsp;axes<sup>1</sup>|Ethernet|EEPROM         |SD&nbsp;card|I2C Keypad|Encoders|Digital I/O|Analog I/O|
|---------------------------------------------------------------------------------|------|----------------------------|--------|---------------|------------|----------|--------|-----------|----------|
|Generic                                                                          | 3    |no                          |no      |yes<sup>2</sup>|yes         |yes       | -      | -         | -        |
|[BOARD_T40X101](https://github.com/phil-barrett/grbl-teensy-4) for Teensy 4.0    |max 4 |max 1                       |no      |yes<sup>2</sup>|no          |yes       | max 1  | -         | -        |
|[BOARD_T41U5XBB](https://github.com/phil-barrett/grbl-teensy-4) for Teensy 4.1   |max 5 |max 2                       |yes     |yes<sup>2</sup>|yes         |yes       | max 1  |4/3 or 1/3<sup>3</sup>|-|
|[BOARD_T41BB5X_PRO](https://github.com/phil-barrett/grbl-teensy-4) for Teensy 4.1|max 5 |max 2                       |yes     |yes \(FRAM\)   |yes         |yes       | max 1  |4/3 or 1/3<sup>3</sup>|-|

<sup>1</sup> Each enabled reduces N_AXIS with one. Currently the board map file must be edited to enable ganged/auto squared axes.  
<sup>2</sup> I<sup>2</sup>C EEPROM \(or FRAM\) is [optional](https://github.com/grblHAL/Plugin_EEPROM/blob/master/README.md) and must be added to the board. FRAM is recommended when the [Odometer plugin](https://github.com/grblHAL/Plugin_odometer/blob/master/README.md) is added to the build.  
<sup>3</sup> Number of digital input pins available is reduced when the [Encoder plugin](https://github.com/grblHAL/Plugin_encoder/blob/master/README.md) is added to the build.

---
2021-06-08
