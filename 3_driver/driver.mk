# choose the external sensors in driver src
DRIVERSRC = \
            $(CHIBIOS)/3_driver/judge.c     \
            $(CHIBIOS)/3_driver/adis16265.c \
            $(CHIBIOS)/3_driver/mpu6500.c
            # $(CHIBIOS)/3_driver/hcsr04.c  \
            # $(CHIBIOS)/3_driver/ist8310.c

# Required include directories
DRIVERINC = $(CHIBIOS)/3_driver/inc