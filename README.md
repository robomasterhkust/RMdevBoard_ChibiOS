# RMdevBoard_ChibiOS  

HKUST RoboMaster ENTERPRIZE team embedded development environment.

## Branches
- Master: For RM2017 board on gimbal only. Adopted from RMInfantry2018 to support the visual servo controller on ros_environment branch. Maintained by Beck Pang.
- cv_chassis: To support the visual servo controller on ros_environment branch. Maintained by Beck Pang.
- Sentry: Old code for RM2017 board, developed by Alex Wong and Alex Au
- new_chassis: Old code for RM2018 board on the competition, developed by Kuang Zeng and TJ.


## Dependency
- compiler: arm-none-eabi-gcc
- debugger: openocd for ST-Link / J-Link for Ozone
- development OS: Ubuntu / MacOS / Windows

## Software Architecture
- os
- hw
- driver
- dev
- module
- task
- tools
- docs

Architecture explained:Ma
- os:   chibiOS / RT, real time operating system;
- hw:  hardware, board related documentation;
- dev: devices, drivers, and external libraries;
- driver: register map and basic functions for external hardwares;
- module: estimator, controller, system identification, and other math related algorithm;
- task: application layers, defining the tasks independent to the hardware.

## Dependency
MacOS use homebrew to install the embedded toolchain
```
brew tap PX4/px4
brew install px4-dev
```

Ubuntu
```
sudo apt-get install gcc-arm-none-eabi openocd -y
```
