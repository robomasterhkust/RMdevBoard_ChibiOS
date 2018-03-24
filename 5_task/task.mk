# choose the tasks in your board to run
TASKSRC =   \
            $(CHIBIOS)/5_task/canBusProcess.c \
            $(CHIBIOS)/5_task/chassis.c \
            $(CHIBIOS)/5_task/error.c \
            # $(CHIBIOS)/5_task/mavlink_comm.c \
            # $(CHIBIOS)/5_task/mavlink_topic.c

# Required include directories
TASKINC =   $(CHIBIOS)/5_task/inc \
            $(CHIBIOS)/5_task/mavlink \
            $(CHIBIOS)/5_task/mavlink/minimal