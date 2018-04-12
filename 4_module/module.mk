# choose the controller and estimator modules you want to use
MODULE_PATH = $(CHIBIOS)/4_module

include $(MODULE_PATH)/dsp/dsp.mk

MODULESRC =    $(MATHSRC)\
               $(MODULE_PATH)/attitude.c \
               $(MODULE_PATH)/attitude_estimator_mpu6500.c \
               $(MODULE_PATH)/math_misc.c \
               $(MODULE_PATH)/gimbal.c
               # $(MODULE_PATH)/gimbal_simple_controller.c

# $(MODULE_PATH)/gimbal_sys_iden.c

# Required include directories
MODULEINC = $(MODULE_PATH)/inc