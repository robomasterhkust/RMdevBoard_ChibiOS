# stable internal files basic hardware functions
DEVSRC =    \
            $(CHIBIOS)/dev/shellcfg.c \
            $(CHIBIOS)/dev/dbus.c \
            $(CHIBIOS)/dev/exti.c \
            $(CHIBIOS)/dev/flash.c \
            $(CHIBIOS)/dev/imu_temp.c \
            $(CHIBIOS)/dev/params.c \
            $(CHIBIOS)/dev/pwm_struct.c \
            $(CHIBIOS)/dev/shoot_pwm.c \
            $(CHIBIOS)/dev/usbcfg.c

            # $(CHIBIOS)/dev/buzzer.c \
            # $(CHIBIOS)/dev/tft_display.c \


# general configuration and headers
DEVINC =    $(CHIBIOS)/dev/inc \
            $(CHIBIOS)/dev/conf