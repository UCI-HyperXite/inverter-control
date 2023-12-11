# Inverter Control

C++ code to generate a three-phase sinusoidal pulse density modulation (SPDM)
signal on a Raspberry Pi Pico using the Pico SDK.

## Setup

To be able to build the Pico executables on your own machines, please follow
the below instructions.

### Necessary Software

For all operating systems, please have VSCode installed. While it is very much
possible to use a different editor (e.g. Neovim), VSCode provides a much easier
configuration that expedites the setup process.

#### Windows

For Windows machines, a convenient installation tool is provided that installs
all the necessary software for you. See [this article](https://www.raspberrypi.com/news/raspberry-pi-pico-windows-installer/) to download the installer.

#### MacOS

For MacOS, a manual installation is required. Please make sure that Homebrew, a
package manager for MacOS, is installed. If not, you can install it by running

```shell
$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Then, run the following commands to install the rest of the required software:

```shell
$ brew install cmake
$ brew tap ArmMbed/homebrew-formulae
$ brew install arm-none-eabi-gcc
```

#### Linux

The process of installation varies depending on the Linux distribution being used.
For example, there exists a [Pico SDK package](https://aur.archlinux.org/packages/pico-sdk)
on the Arch Linux User Repository.

For Ubuntu/Debian-based Linux, however, no such installer exists. Please run
the following commands to install the necessary libraries.

```shell
$ sudo apt update
$ sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential libstdc++-arm-none-eabi-newlib
```

If you do not have a C/C++ compiler installed already, also run:

```shell
$ sudo apt install build-essential
```

and to verify that the compilers have been installed, execute:

```shell
$ gcc --version
$ g++ --version
```

Version numbers should appear as output from both commands.

### VSCode Configuration

After setting up the Pico SDK, please open VSCode and install the following
extensions. Note that while these are not explicitly required, they greatly
enhance developer experience:

- C/C++ Extension Pack (C++ Intellisense)
- CMake (CMake Intellisense)
- CMake Tools (for easier interactions involving building with CMake)
- Makefile Tools (Makefile Intellisense)

To configure CMake, press `Ctrl/cmd + Shift + p` and enter
`CMake: Select a Kit`. This should prompt you to choose which compilers you
want to use for this project. Select the one that contains `arm-none-eabi`,
since these are needed to be able to compile the Pico executables properly.

Now load VSCode in this folder and open `src/main.cpp`. If you see errors
related to an `#include` header not being found, hover over it, click
"Quick Fix" and then "Edit include path settings". You can also access these
settings by pressing `Ctrl/Cmd + Shift + p` and then entering
`C/C++: Edit Configurations (UI)`. Scroll down to the "Include Path" setting
and append `${env:PICO_SDK_PATH}/**` to it. C/C++ Intellisense should
automatically update with these new settings and the errors should disappear.

## Building

Now that everything has been installed, it's time to build the program!
Specifically, we will generate the UF2 executable that the Pico needs and
manually copy this file over to the Pico.

If you have the CMake extension installed, you can click "Build" on the
bottom. The executable files for the Pico, including the UF2, will be
automatically generated in the `build` directory.

If you'd like to run the commands themselves, you can run the following:

```shell
$ cmake -B build .
$ cd build
$ make
```

If errors occur, try restarting your terminal and then running the
commands again.

## Running the Code

Now that the UF2 file has been generated, connect the Pico to your computer
while holding down the white BOOTSEL button. If the Pico is already connected,
disconnect it and reconnect it while holding down the BOOTSEL button.

Copy over the UF2 and the Pico should start running your code!
