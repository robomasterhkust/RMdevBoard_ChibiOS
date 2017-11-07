# RMdevBoard_ChibiOS  
  
## Get Chibi up and running in Windows
(Written by Alex Wong, tell me if I got sth wrong)  
  
1. Downloading ChibiStudio  
  - Go to [ChibiStudio download site](https://sourceforge.net/projects/chibios/files/ChibiStudio/) and download the latest version of ChibiStudio. At the time of writing, latest version available was ChibiStudio_Preview19.7z  
  
  
2. Installing ChibiStudio  
  - Extract and place the ChibiStudio folder into:  
  > C:\  
  - Such that your Chibistudio installation path is  
  > C:\ChibiStudio  
  
  
3. Copy launch shortcut elsewhere
  - Inside 'C:\ChibiStudio', you will see two shortcuts, "Chibi Studio GCC 4.7" and "Chibi Studio GCC 6.3", we will be using the one ending with 6.3, so copy and paste that shortcut to somewhere convenient for you.  
  
  
4. Launch Eclipse for the first time  
  - Eclipse will prompt you to choose a location as your workspace, enter:   
  > C:\ChibiStudio\workspace_user   
  - Eclipse will launch and show a welcoming 'readme.txt'  
  
  
5. Importing this project  
  - Git clone this repo into  
  > C:\ChibiStudio\workspace_user\RMdevBoard_ChibiOS  
  - Launch Eclipse and select 'File>Import'   
  - In the 'Import' window, choose 'Existig Projects into Workspace' and press 'Next'  
  - Choose 'Select root directory', have   
  > C:\ChibiStudio\workspace_user\RMdevBoard_ChibiOS  
    as your root directory  
  - You only need to import "3300Proj", don't import "STM32F4xx-CAN"  
  - leave the three options un-checked  
  - Press 'Finish'  
  
  
6. Testing Everything
  - Press 'Ctrl-B' on your keyboard
  - If everything is correct, the console should display a bunch of source files being compiled, follwed by 'Done' at the end, ignore the warnings  
  
  
7. Update Eclipse Version (optional)  
  ChibiStudio comes with Eclipse LUNA, which you can only theme the text editor dark, not the rest of the UI like Project explorer, console etc. If you are OCD like me, you can consider updating Eclipse to a newer release, at the time of writing, latest version available was Eclipse OXYGEN  
  - Go into 'C:\ChibiStudio' and either rename or delete your eclipse folder  
  - Goto the [Eclipse download site](http://www.eclipse.org/downloads/eclipse-packages/) and download **Eclipse IDE for C/C++ Developers** for your windows installation  
  - Extract and place your new eclipse folder into 'C:\ChibiStudio'  
  - You can theme your eclipse dark by going to 'Help>Eclipse Marketplace', then search and install 'Darkest Dark Theme 2017', it is quite nice  
  
  
8. Flashing compiled code into the RM board's MCU (using J-Link)  
  - Download and install [Ozone](https://www.segger.com/downloads/jlink/#Ozone) if you haven't already  
  - Launch Ozone  
  - In the 'New Project Wizard', choose 'STM32F427II' as 'Device', press 'Next >'  
  - 'Target Interface' = 'SWD'; 'Target Interface Speed' = '50MHz'; 'Host Interface' = 'USB', press 'Next >'  
  - For 'ELF, Motorola S-record .....' field, browse and choose  
  > C:/ChibiStudio/workspace_user/RMdevBoard_ChibiOS/dev/build/ch.elf  
  - After project creation, at the top-left corner, press the 'Download and reset program' button, which is shaped like a green power button  
  - After flashing the program, press the triangle-shaped 'Resume program excution' button  
  - The green LED on the RM board should flash on and off every 500ms if everything is correct  
  
## Get ChibiOS up and running in Mac [provisory]
This installation makes use of terminal, if you are not familiar with the use of the command line tool you can follow this [tutorial](https://www.davidbaumgold.com/tutorials/command-line/) to get started with it:
 
1. Open terminal and create a directory inside your user folder and name it GitHub with the following command

		mkdir GitHub

2. cd into the directory with 

		cd GitHub

3. Clone this repository

		git clone https://github.com/robomasterhkust/RMdevBoard_ChibiOS

4. Open CLion (if you do not have CLion installed [follow these steps](#install-clion-with-a-free-student-license) to obtain a free student license) or any other editor of choice and import `RMdevBoard_ChibiOS` as a project.

5. You can now modify all the code, however in order for new `.c` and `.h` files to be compiled you will need to add them to the make file. Under the section '# C sources'. For example in order to add a `test.c` file after creating it and adding it into the 'dev' folder (with relative header `test.h` in the 'inc' folder) the following section:
```
CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(TESTSRC) \
			 $(CHIBIOS)/os/various/shell.c \
       $(CHIBIOS)/os/various/evtimer.c \
       $(CHIBIOS)/os/various/syscalls.c \
			 $(CHIBIOS)/os/hal/lib/streams/memstreams.c \
       $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
       main.c shellcfg.c flash.c tft_display.c mpu6500.c math_misc.c \
			 canBusProcess.c dbus.c gimbal.c params.c
```
  	Will become:

```
CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(TESTSRC) \
			 $(CHIBIOS)/os/various/shell.c \
       $(CHIBIOS)/os/various/evtimer.c \
       $(CHIBIOS)/os/various/syscalls.c \
			 $(CHIBIOS)/os/hal/lib/streams/memstreams.c \
       $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
       main.c shellcfg.c flash.c tft_display.c mpu6500.c math_misc.c \
			 canBusProcess.c dbus.c gimbal.c params.c test.c
```

6. Flash compiled code into the board (two methods):
  - Command Line, using STLINKV2 (looks like a USB stick with 8 Pins on the end):
    - In terminal cd into the dev directory:
    
          cd GitHub/RMdevBoard_ChibiOS/dev
    - type
    
          make upload
     - The green LED should be blinking once every 500ms, SUCCESS
  - Ozone using JLINK (same used for internals):
    - In terminal, cd into the dev directory:
    
          cd GitHub/RMdevBoard_ChibiOS/dev
    - type
    
          make
      this step is necessary only if you changed the code since the last version
    - Download and install [Ozone](https://www.segger.com/downloads/jlink/#Ozone) if you haven't already  
    - Launch Ozone  
    - Select 'New Project', choose 'STM32F427II' as 'Device', press 'Next >'  
    - 'Target Interface' = 'SWD'; 'Target Interface Speed' = '50MHz'; 'Host Interface' = 'USB', press 'Next >'  
    - For 'ELF, Motorola S-record .....' field, browse into the GitHub folder and choose  
    `User/`your-username`/GitHub/ChibiStudio/workspace_user/RMdevBoard_ChibiOS/dev/build/ch.elf
    - After project creation, at the top-left corner, press the 'Download and reset program' button, which is shaped like a green power button  
    - After flashing the program, press the triangle-shaped 'Resume program excution' button  
    - The green LED on the RM board should flash on and off every 500ms if everything is correct


### Install CLion with a free student license
 1. Go to [JetBrains website](https://www.jetbrains.com/clion/download/#section=mac) and dowload CLion
 2. Obtain a student license using [this procedure](https://www.jetbrains.com/shop/eform/students) and your connect.ust.hk email address
  
  
