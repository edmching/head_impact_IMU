# Head Impact IMU Firmware
This is the project folder for the IMU firmware. It includes the nRF5 SDK 15.2.0 source files and the nrf52832_mdk development board config files. 

## Setting up breadboard development environment
Requirements:
* GNU Arm Embedded Toolchain
* GNU make [GNUwin Make (for Windows)](http://gnuwin32.sourceforge.net/) 
* pyOCD

## Getting Started

### Downloading Git (For Windows Users Only)
Download Git for Windows https://git-scm.com/downloads to clone this repository. We will use Git Bash terminal for building this project. 

Clone this repo by using:
``` sh
git clone
https://github.com/edmching/head_impact_IMU
```

### Setting up GNU ARM Embedded Toolchain
Download and install the latest version of [GNU Arm Embedded Toolchain ](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads). Then make sure to add the path to your toolchain to your OS "PATH" environment variable:

``` sh
<path to install directory>/gcc-arm-none-eabi-6-2017-q2-update/bin
```

Adding the path makes it possible to run the toolchain executables from any directory using the terminal. To verify that the path is set correctly, type the following in your terminal:

``` sh
$ arm-none-eabi-gcc --version
```

### Setting up GNUMake

For Windows download and install [GNUwin Make (for Windows)](http://gnuwin32.sourceforge.net/packages/make.htm). Download the "Complete package, except sources	 	Setup" link. Run "make-3.81.exe" to install make. Add the path to your GNU Make executable to your "PATH" environment variable. For example:

``` sh
C:\Program Files (x86)\GnuWin32\bin
```

Check to see it works by using 

``` sh
$ make
make: *** No targets specified and no makefile found.  Stop.
 ```

### Setting up pyOCD
[pyOCD](https://github.com/mbedmicro/pyOCD) is an open source Python library for programming and debugging for ARM Cortex-M devices.

* Requires Python 2.7.9 or later, or Python 3.6.0 or later.

* **Workaround for libusb error on Python 3.7 on Windows: Use Python 3.8**

Download and install Python 3.8 from https://www.python.org/. For Windows user, please check the "Add to PATH" box when installing.

Use pip to install pyOCD. 

``` sh
 $ pip install --pre -U pyocd
```

Check to see if installed correctly:

``` sh
 $ pyocd
```

## Building the Project

Once you have the GNU ARM toolchain, GNU make and pyOCD setup, you can build this project. 

The `makefile` is located in  `<your path >/head_impact_IMU/ble_app/armgcc/ .

Go to your app config folder and run make:
``` sh
$ cd ble_app/armgcc
$ make flash
```
The command `make` will compile the and link the source files. 
If successful you should see:

``` sh
$ make
Linking target: _build/nrf52832_xxaa.out
   text    data     bss     dec     hex filename
  26604     164    2464   29232    7230 _build/nrf52832_xxaa.out
Preparing: _build/nrf52832_xxaa.hex
Preparing: _build/nrf52832_xxaa.bin
DONE nrf52832_xxaa
```

 If you want to flash to your development board run `make flash` and you should see:

```sh
 $ make flash
DONE nrf52832_xxaa
Flashing: _build/nrf52832_xxaa.hex
pyocd-flashtool -t nrf52 -se _build/nrf52832_xxaa.hex
WARNING:pyocd.tools.flash_tool:pyocd-flashtool is deprecated; please use the new combined pyocd tool.
INFO:pyocd.board.board:Target type is nrf52
INFO:pyocd.coresight.dap:DP IDR = 0x2ba01477 (v1 rev2)
INFO:pyocd.coresight.ap:AP#0 IDR = 0x24770011 (AHB-AP var1 rev2)
INFO:pyocd.coresight.ap:AP#1 IDR = 0x02880000 (proprietary)
INFO:pyocd.coresight.rom_table:AP#0 ROM table #0 @ 0xe00ff000 (designer=244 part=006)
INFO:pyocd.coresight.rom_table:[0]<e000e000:SCS-M4 class=14 designer=43b part=00c>
INFO:pyocd.coresight.rom_table:[1]<e0001000:DWT class=14 designer=43b part=002>
INFO:pyocd.coresight.rom_table:[2]<e0002000:FPB class=14 designer=43b part=003>
INFO:pyocd.coresight.rom_table:[3]<e0000000:ITM class=14 designer=43b part=001>
INFO:pyocd.coresight.rom_table:[4]<e0040000:TPIU-M4 class=9 designer=43b part=9a1 devtype=11 archid=0000 devid=0:0:ca1>
INFO:pyocd.coresight.rom_table:[5]<e0041000:ETM-M4 class=9 designer=43b part=925 devtype=13 archid=0000 devid=0:0:0>
INFO:pyocd.coresight.cortex_m:CPU core #0 is Cortex-M4 r0p1
INFO:pyocd.coresight.cortex_m:FPU present: FPv4-SP
INFO:pyocd.coresight.dwt:4 hardware watchpoints
INFO:pyocd.coresight.fpb:6 hardware breakpoints, 4 literal comparators
[---|---|---|---|---|---|---|---|---|----]
[========================================]
INFO:pyocd.flash.loader:Erased 0 bytes (0 sectors), programmed 0 bytes (0 pages), skipped 28672 bytes (7 pages) at 9.19 kB/s
```
## Text Editor
You may use any text editor you like, however we chose [Visual Studio Code](https://code.visualstudio.com/) since it provided a debugging environment. 
We used the following extensions in VS Code:
* [C/C++ IntelliSense](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
* [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug)

A `launch.json` file is included in .vscode folder for setting up the pyocd debugger and jlink debugger.
`launch.json` is for debugging PCB firmware and `launch_pyocd.json` is for debugging breadboard firmware. If you wish to debug breadboard firmware you must rename the `launch_pyocd.json` to `launch.json` and vice versa.

Another note is the jlink `launch.json` is set up for a linux user so you will have to change the path locations for "serverpath" and "svdFile" in the launch.json.

```sh
        "serverpath": "/opt/SEGGER/JLink/JLinkGDBServerCLExe", //change to your path
        .
        .
        .
        "svdFile": "/opt/SEGGER/Ozone/Config/CPU/Cortex-M4F.svd" //change to your path
```
**See reference links at the end on a guide to setup your launch.json**

## Receiving Data Serially

Nearly all of the project code prints messages or data serially. In order to receive this data, an SSH client is required. We used [PuTTY](https://www.putty.org/).

Once you have downloaded PuTTy, you will need to set up the serial port. Make sure you know which serial port the device is using (i.e. COM9).

The default setup for the USB serial port is 115200 baud, 8 bits, 1 stop bit, no parity (115200-8-N-1) - you will need to input this information into your SSH client in order to communicate.

More information on using terminal with the development board can be found [here](https://wiki.makerdiary.com/nrf52832-mdk/getting-started/).

## Setting up PCB development environment
Software Requirements:
* GNU Arm Embedded Toolchain
* GNU make [GNUwin Make (for Windows)](http://gnuwin32.sourceforge.net/) 
* nRF Command Line Tools

Assuming you have GNU arm toolchain and make setup properly from the above.

Hardware Requirements:
* PCB REV
* JLink mini EDU + USB A to micro USB B cable
* USB to UART adapter + USB A to USB B cable
* LiPo battery (optional if you are using the USB to UART adapter to power the device)

The USB to UART adapter is for getting serial output from the tx and rx pins. You may use the VCC and GND of the USB to UART adapter to power the PCB. The connections are VCC->VBAT, GND->GND, Tx->Rx, Rx->Tx.

## Installing nRF Command Line Tools
Download and install [nRF Command Line Tools](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Command-Line-Tools). By default, the installer should include it in the path environment variable. Check that the path environment variable is set like:

``` sh
C:\Program Files\Nordic Semiconductor\nrf-command-line-tools\bin\
 ```

Now instead of using `pyocd` as the programmer and debugger, the `nrfjprog` will be the programmer and debugger. The type of programmer should already be included in the makefile.

The Jlink is for programming and debugging firmware on the PCB. JLink requires using the `nrfjprog` command from nRF Command Line Tools package instead of `pyocd`. The JLink has a ribbon cable connector included. To check if the connection is correct, you can run `JLinkExe` and check that the `VTref is ~3.3V` and that you can connect to the MCU. See below for an example output.

```sh
> JLinkExe
SEGGER J-Link Commander V6.62a (Compiled Jan 31 2020 12:58:00)
DLL version V6.62a, compiled Jan 31 2020 12:57:49

Connecting to J-Link via USB...O.K.
Firmware: J-Link EDU Mini V1 compiled Jan  7 2020 16:53:19
Hardware version: V1.00
S/N: 801011674
License(s): FlashBP, GDB
VTref=3.299V


Type "connect" to establish a target connection, '?' for help
J-Link>connect
Please specify device / core. <Default>: NRF52832_XXAA
Type '?' for selection dialog
Device>
Please specify target interface:
  J) JTAG (Default)
  S) SWD
  T) cJTAG
TIF>S
Specify target interface speed [kHz]. <Default>: 4000 kHz
Speed>
Device "NRF52832_XXAA" selected.


Connecting to target via SWD
InitTarget() start
InitTarget() end
Found SW-DP with ID 0x2BA01477
DPIDR: 0x2BA01477
Scanning AP map to find all available APs
AP[2]: Stopped AP scan as end of AP map has been reached
AP[0]: AHB-AP (IDR: 0x24770011)
AP[1]: JTAG-AP (IDR: 0x02880000)
Iterating through AP map to find AHB-AP to use
AP[0]: Core found
AP[0]: AHB-AP ROM base: 0xE00FF000
CPUID register: 0x410FC241. Implementer code: 0x41 (ARM)
Found Cortex-M4 r0p1, Little endian.
FPUnit: 6 code (BP) slots and 2 literal slots
CoreSight components:
ROMTbl[0] @ E00FF000
ROMTbl[0][0]: E000E000, CID: B105E00D, PID: 000BB00C SCS-M7
ROMTbl[0][1]: E0001000, CID: B105E00D, PID: 003BB002 DWT
ROMTbl[0][2]: E0002000, CID: B105E00D, PID: 002BB003 FPB
ROMTbl[0][3]: E0000000, CID: B105E00D, PID: 003BB001 ITM
ROMTbl[0][4]: E0040000, CID: B105900D, PID: 000BB9A1 TPIU
ROMTbl[0][5]: E0041000, CID: B105900D, PID: 000BB925 ETM
Cortex-M4 identified.
J-Link>

```

## Navigating the Repository

### head_impact_IMU

This is the repo's root directory. You can access everything you need starting from here. If you need to edit this README file, you can find it here.
There are also three main sub-directories: 
* nrf_sdk
* board_config
* ble_app

### nrf_sdk

This directory contains part of the Nordic Semiconductor Software Development Kit (SDK). The full version is an extremely detailed guide for developing applications on the NRF52832 microcontroller used by the project.

The partial version contained here consists mainly of toolchain resources (as you have seen above), as well as various libraries that are included throughout the project.

[The full SDK](https://www.nordicsemi.com/Software-and-tools/Software/nRF5-SDK) contains various examples, most of which are irrelevant to this project, and so are not included here. Nordic Semiconductor also has an online[Infocenter](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.0.0%2Fexamples.html) that provides more detailed information on the SDK and its various modules.

### board_config

This directory contains board configuration files for both currently implemented platforms: PCB Revision 1 (*imu_pcb_rev1*) and the breadboard + MCU development (*nrf52832_mdk*).

These files describe the pin mapping for each of these specific platforms - they are fairly different, so make sure you know which platform you are using and that the code you are running aligns with that platform. One of these files will be #included at the top of the code you are running.

**Note:** running code on a different platform than it was written for is not always as easy as changing these #included files, especially when SPI drivers are involved. PCB Revision 1 uses different SPI drivers than the breadboard set-up - refer to the commented driver files for more information.

A template (*custom_board*) is included so that pin mapping can be done for future platforms.

### ble_app

This directory contains all of the code developed so far for the IMU project. All new code should be added to one of the sub-directories located here (or in a new sub-directory).

#### test

This directory contains all of the test code written throughout to project (which is the bulk of the code). Each sub-directory contains code to test a specific component of the IMU. For example, the *adxl372_test* directory contains all of the code relevant to communicate with and receive measurements from the ADXL372 accelerometer. Running code such as this ensures the functionality of each peripheral, and can be used as reference when integrating multiple peripherals together.

Each sub-directory contains source code (.c, .h) that, in general, initializes a single, specific peripheral, configures it, and prints messages or data to serial to showcase its functionality. Additionally, each sub-directory includes a makefile and sdk_config file, which are discussed later.

It is highly recommnded that any substantive new code begin its life here, prior to being introduced to any integrated code.

#### integration

The code located here brings the device peripherals together and offers integrated functionality. The *sensors_integration* code was developed for the breadboard platform, while the *imu_pcb_rev1* was developed for PCB Revision 1.

#### drivers

This directory contains the driver files that are called by the SPI and I2C peripherals. Note that the two current platforms utilize different SPI drivers - see the driver file comments for more information.

These files are **NOT** intended to be run on their own, and therefore do not have dedicated makefiles or config files - their functions are heavily used throughout the test and integration code sets.

#### config

This directory contains the default *sdk_config.h* file. This file type is discussed in more detail later.

#### ble_app_cli

This directory contains the code for the Bluetooth command line interface.

## Adding Additional Code

Each new code set requires a *sdk_config.h* file and *makefile* within its directory, in addition to the basic source code.

### sdk_config file

This header file allows you to modify the configuration options for the code that you write. Essentially, it contains a list of #define statements that allow you to select the MCU features or specifc modes used by your application.

For continued functionality of the applciations we've developed on these platforms, we suggest copy-and-pasting the sdk_config file used by the current code set that is most comparable to your new code set, and using that as a basis for your new configuration file. In the unlikely event that you need to modify your configurations, you can edit this file.

Nordic Semiconductors supplies more information on these configuration files [here](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.0.0%2Fsdk_config.html)

### makefile

A makefile is required so that the make command can compile and link your program. This allows the make command to create the source files required for running code on the device. All of the source files and folders common to the make target need to be included in the makefile in order for the program to be compiled succesfully.

Once again, creating a copy of a current makefile and modifying it for your use is the easiest way of creating a new makefile. 

The GNU make manual can be found [here](https://www.gnu.org/software/make/manual/make.html)

## Reference Links
* [Makerdiary setting up development environment](https://wiki.makerdiary.com/nrf52832-mdk/nrf5-sdk/)
* [Makerdiary examples to run on development board](https://github.com/makerdiary/nrf52832-mdk)
* [Vscode debugging on breadboard using pyocd](https://store.makerdiary.com/blogs/tutorials/debugging-nrf52840-mdk-board-with-visual-studio-code)
* [Vscode debugging on PCB Rev1 using JLink](https://wiki.segger.com/J-Link:Visual_Studio_Code)
* [Nordic SDK 15.2 documentation](https://store.makerdiary.com/blogs/tutorials/debugging-nrf52840-mdk-board-with-visual-studio-code)
