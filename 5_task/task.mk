# choose the tasks in your board to run
TASK_PATH = $(CHIBIOS)/5_task

TASKSRC =   \
            $(TASK_PATH)/chassis.c \
            $(TASK_PATH)/detect_error_task.c \
            $(TASK_PATH)/attitude_estimator_task.c \
            $(TASK_PATH)/can_motor_task.c \
            $(TASK_PATH)/can_communication_task.c \
            $(TASK_PATH)/command_mixer_task.c
            # $(TASK_PATH)/canBusProcess.c \
            # $(TASK_PATH)/error.c \
            # $(TASK_PATH)/mavlink_comm.c \
            # $(TASK_PATH)/mavlink_topic.c

# Required include directories
TASKINC =   $(TASK_PATH)/inc \
            $(TASK_PATH)/mavlink \
            $(TASK_PATH)/mavlink/minimal