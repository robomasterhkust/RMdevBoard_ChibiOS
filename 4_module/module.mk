# choose the controller and estimator modules you want to use
MODULESRC =    \
               $(CHIBIOS)/4_module/attitude.c \
               $(CHIBIOS)/4_module/math_misc.c \
               $(CHIBIOS)/4_module/gimbal.c
# \ $(CHIBIOS)/4_module/gimbal_sys_iden.c

# Required include directories
MODULEINC = $(CHIBIOS)/4_module/inc