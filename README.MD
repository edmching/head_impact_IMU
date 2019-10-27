#Head Impact IMU firmware

#Introduction
The base project folder for the IMU firmware. It includes the nRF SDK source files and the nrf52832_mdk development board config files. 

* To build this project we need:
    * GNU Arm Embedded Toolchain
    * GNU make [GNUwin Make (for Windows)](http://gnuwin32.sourceforge.net/) 
    * pyOCD

## Getting Started

### For Windows Users Only
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

Now you make set the path to your toolchain in the `makefile.windows` or `makefile.posix` depending on your OS. This is located in 

``` sh
nrf_sdk/components/toolchain/gcc
```

Using a text editor set the `GNU_INSTALL_ROOT` and `GNU_VERSION`variables. For example:

``` sh
GNU_INSTALL_ROOT := <path to install directory>/gcc-arm-none-eabi-8-2019-q3-update-win32/bin/
GNU_VERSION := 8.3.1
GNU_PREFIX := arm-none-eabi
```

**Where make sure you use '/' or make will not be able too find your direction!**

### Setting up GNUMake

For Windows download and install [GNUwin Make (for Windows)](http://gnuwin32.sourceforge.net/). Add the path to your GNU Make executable to your "PATH" environment variable. For example:

``` sh
C:\Program Files (x86)\GnuWin32\bin
```

Check to see it works by using:

``` sh
make
```

You should get the following message:
 ``` sh
 make: *** No targets specified and no makefile found.  Stop.
 ```

### Setting up pyOCD
[pyOCD](https://github.com/mbedmicro/pyOCD) is an open source Python library for programming and debugging for ARM Cortex-M devices. It requires Python 2.7.9 or later, or Python 3.6.0 or later.

Download and install Python from https://www.python.org/. For windows user, please check the "Add to PATH" box when installing.

We will use Pip to install pyOCD. 

``` sh
pip install --pre -U pyocd
```

Check to see if installed correctly:

``` sh
pip
```

## Building the Project

Once you have the GNU ARM toolchain, GNU make and pyOCD setup, you can build this project. 

The `makefile` is located in  `<your path >/head_impact_IMU/ble_app/armgcc/ .

Go to the app config folder and run make:
``` sh
cd ble_app/armgcc
make flash
```
The command `make` will compile the and link the source files.

If successful you should see:

``` sh
Linking target: _build/nrf52832_xxaa.out
   text    data     bss     dec     hex filename
  26604     164    2464   29232    7230 _build/nrf52832_xxaa.out
Preparing: _build/nrf52832_xxaa.hex
Preparing: _build/nrf52832_xxaa.bin
DONE nrf52832_xxaa
```

 If you want to flash to your development board run `make flash`. 