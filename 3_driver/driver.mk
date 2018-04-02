# choose the external sensors in driver src
DRIVER_PATH = $(CHIBIOS)/3_driver

DRIVERSRC = \
            $(DRIVER_PATH)/judge.c     \
            $(DRIVER_PATH)/adis16265.c \
            $(DRIVER_PATH)/mpu6500.c
            # $(DRIVER_PATH)/hcsr04.c  \
            # $(DRIVER_PATH)/ist8310.c \
            # $(DRIVER_PATH)/tft_display.c \

# Required include directories
DRIVERINC = $(DRIVER_PATH)/inc