# use the external tools for debugging
TOOL_PATH = $(CHIBIOS)/6_tool

TOOLSRC =   \
            $(TOOL_PATH)/calibrate_sensor.c
# $(TOOL_PATH)/sdlog.c \
# $(TOOL_PATH)/system_identification.c


# Required include directories
TOOLINC =   $(TOOL_PATH)/inc