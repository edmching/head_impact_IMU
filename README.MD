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
## Setting up PCB development environment
Requirements:
* GNU Arm Embedded Toolchain
* GNU make [GNUwin Make (for Windows)](http://gnuwin32.sourceforge.net/) 
* nRF Command Line Tools

Assuming you have GNU arm toolchain and make setup properly

## Installing nRF Command Line Tools
Download and install [nRF Command Line Tools](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Command-Line-Tools). By default, the installer should include it in the path environment variable. Check that the path environment variable is set like:

``` sh
C:\Program Files\Nordic Semiconductor\nrf-command-line-tools\bin\
 ```
Now instead of using pyocd as the programmer and debugger, the nrfjprog will be the programmer and debugger. The type of programmer should already be included in the makefile.


## Reference Links
* [Makerdiary setting up development environment](https://wiki.makerdiary.com/nrf52832-mdk/nrf5-sdk/)
* [Makerdiary examples to run on development board](https://github.com/makerdiary/nrf52832-mdk)
* [Setting up visual studio code debugger](https://store.makerdiary.com/blogs/tutorials/debugging-nrf52840-mdk-board-with-visual-studio-code)
* [Nordic SDK 15.2 documentation](https://store.makerdiary.com/blogs/tutorials/debugging-nrf52840-mdk-board-with-visual-studio-code)

