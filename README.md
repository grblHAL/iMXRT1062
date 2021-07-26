## iMXRT1062 Driver

A grblHAL driver for the NXP iMXRT1062 processor on a [Teensy 4.x board](https://www.pjrc.com/store/teensy40.html).

Available driver options can be found [here](main/my_machine.h).

See [Compiling](#Compiling) for more information on building.

---

__Important!__ There is a "bug" in Teensyduino prior to v1.54 that may cause [periodic stalls](https://github.com/grblHAL/iMXRT1062/issues/6) in processing.
It is possible that this is only happening when networking is enabled and then not always so.  
Regardless of whether networking is enabled or not it is recommended that [Teensyduino v1.54](https://www.pjrc.com/teensy/td_download.html) is used to build this driver.

---

#### Networking plugin

The networking plugin is for Teensy 4.1 and needs the [teensy41_ethernet lwIP library](https://github.com/ddrown/teensy41_ethernet) forked by ddrown.

Telnet, websocket and ftp protocols are currently supported, http is on the long term roadmap.

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

### Compiling

grblHAL can be built using the Arduino IDE or through the use of PlatformIO.
Detailed directions may be found in the [grblHAL
wiki](https://github.com/grblHAL/core/wiki/Compiling-GrblHAL).


#### Arduino IDE 

This driver compiles and uploads from the Arduino IDE and is partially dependent on the Arduino framework. [Teensyduino](https://www.pjrc.com/teensy/td_download.html) is required and must be added to the Arduino IDE.

See the Wiki-page for [compiling grblHAL](https://github.com/grblHAL/core/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.


#### PlatformIO 

##### About
[PlatformIO][PlatformIO] is a cross platform build system for embedded systems.
It provides both a GUI (delivered as an extension for VSCode) as well as a
command line interface, both of which wrap the underlying toolsi (`scons`,
`gdb`, etc).  It features library management, a robust interface for dynamic
builds and scripting, and a set of Python APIs for customization.  Users
interested in exploring complex project configurations utilzing many vendor
provided hardware abstraction layers, processor specific customizations, etc may
consult the configurations used within the Marlin project (configurations may be
found in `platformio.ini` and `ini/*`).

##### Quick Start

Compiling grblHAL with PlatformIO is quite trivial.  PlatformIO will handle
setting up any processor/architecture specific tooling needed to compile and
flash grblHAL.  To begin, decide whether you are choosing to use the GUI via
VSCode or the command line tooling. Consult the [documentation][pio-docs]
for directions on installing in the desired manner.

Next we will clone this repository, ensuring that all submodules have been
retrieved:

```bash
git clone --recurse-submodules https://github.com/grblHAL/iMXRT1062.git
```

Next, change into the `grblHAL_Teensy4` sub-directory located within your checkout
of the project (by default this would be `iMXRT1062/grblHAL_Teensy4`).

This directory contains the `platformio.ini` configuration file.  Within the
configuration file we have some basic boilerplate information specifying how to
build the project.  These settings describe:

  - The `board` we desire to compile for (the Teensy 4.0 or 4.1) Note: Both
    boards are defined in `platformio.ini`.  The primary distinction between the
    boards is the onboard flash size (1.94MB in the Teensy 4.0 and 7.75MB in the
    Teensy 4.1).  While either environment will generally work, using the wrong
    environment may raise errors in the future if the build sizes become too
    large.
  - The `platform` to be used (Within PlatformIO a development platform is
    described as "a particular microcontroller or processor architecture that
    PlatformIO projects can be compiled to run on. (A few platforms, for example
    Teensy, use different target architectures for different boards.)"
  - The `framework` we will use for development (For the Teensy we use
    `arduino`.  Examples of other frameworks inclue `CMSIS`, `FreeRTOS`,
    `STM32Cube`, etc).
  - A working `environment` which scopes specific configurations for the tools
    `pio run`, `pio test`, `pio check`, `pio debug`, and any custom targets
    which may be defined.  Our environment re-uses the board name, `teensy41`
    and sets this value as the default environment.
  - Any 3rd-party libraries we may need (e.g. uSDFS, Ethernet, etc)
  - How assets should be flashed to the device (The `teensy-cli` application)

The configuration file also provides a number of configuration abstractions
where common configurations can be applied project wide or by build environment.
For more information on customizing your configuration or build environment,
consult the [PlatformIO documentation][pio-docs].

Next, make any desired edits to the file `src/my_machine.h`

Begin compilation by running the command:

```bash
pio run
```

This will begin the compilation, using the default environment.  Alternate
environments may be specified using the flag `-e`/`--environment`.  Additional
targets may be viewed by running `pio run --list-targets`.  Changing the target
from the default (compilation) can be done using the flag `-t`/`--target`
(e.g. `pio run -t clean`).

As the compilation begins all of the needed tooling and libraries will be
retrieved.  Tooling will be installed to the user's "global" PlatformIO
installation.  Project specific libraries will be stored in the subdirectory
`.pio`.  The `.pio` directory is solely used for temporary build artifacts and
caching libraries.  It is safe to completely remove and will be re-created on
the next execution of `pio run`.

At the end of compilation, two assets will be generated:
  - `.pio/build/teensy41/firmware.elf`
  - `.pio/build/teensy41/firmware.hex`

Our ELF ([Executable and Linkable Format][elf]) binary contains the full set of
headers desribing our program and section headers.  Our HEX file is the binary
rendering of solely the data section of the ELF file.  The HEX file is the one
used by the default Teensy tooling.  The ELF file is useful when performing
debugging (i.e. through the use of `gdb` or `openocd`).

We may use the target `upload` to flash our new firmware.  The default
project-specific configuration in `platformio.ini` utilizes the Teensy CLI
application.  A complete list of supported upload protocols for the Teensy 4.1
(e.g. `teensy-gui`, `jlink`) can be referenced on the [Teensy 4.1][pio-teensy41]
page in the PlatformIO documentation.

To execute our upload, run the following command:

```bash
pio run -t upload
```

Congratulations!  You should now have a newly flashed Teensy running grblHAL!

##### Updating your check-out

To update your checkout in the future, ensure that all git submodules are
updates along with the primary repository:

```bash
git pull --recurse-submodules
```

[elf]: https://en.wikipedia.org/wiki/Executable_and_Linkable_Format
[Marlin]: https://github.com/MarlinFirmware/Marlin/
[PlatformIO]: https://www.platformio.org
[pio-docs]: https://docs.platformio.org/en/latest/
[pio-teensy41]: https://docs.platformio.org/en/latest/boards/teensy/teensy41.html

---
2021-07-11
