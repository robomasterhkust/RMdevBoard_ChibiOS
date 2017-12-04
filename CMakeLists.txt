set(CMAKE_CXX_STANDARD 11)

cmake_minimum_required(VERSION 3.8)
project(RMdevBoard_ChibiOS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_custom_target(STLink COMMAND make upload -C ${RMdevBoard_ChibiOS_SOURCE_DIR}/dev
        CLION_EXE_DIR=${PROJECT_BINARY_DIR})

add_custom_target(Ozone COMMAND make -C ${RMdevBoard_ChibiOS_SOURCE_DIR}/dev
        CLION_EXE_DIR=${PROJECT_BINARY_DIR})


include_directories(dev)
include_directories(dev/inc)
file(GLOB_RECURSE USER_SRC "dev/*.c")

# include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
file(GLOB_RECURSE STARTUP_SRC "os/common/ports/ARMCMx/compilers/GCC/crt1.c" "os/common/ports/ARMCMx/compilers/GCC/vectors.c")
include_directories(os/common/ports/ARMCMx/compilers/GCC/)
include_directories(os/common/ports/ARMCMx/devices/STM32F4xx)
include_directories(os/ext/CMSIS/include)
include_directories(os/ext/CMSIS/ST/STM32F4xx)

SET(LINKER_SCRIPT os/common/ports/ARMCMx/compilers/GCC/ld/STM32F429xI.ld)

# include $(CHIBIOS)/os/hal/hal.mk
file(GLOB_RECURSE HAL_SRC "os/hal/src/*.c")
include_directories(os/hal/include)

#include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
file(GLOB_RECURSE PLATFORM_SRC
        "os/hal/ports/common/ARMCMx/nvic.c          "
        "os/hal/ports/STM32/STM32F4xx/hal_lld.c     "
        "os/hal/ports/STM32/STM32F4xx/ext_lld_isr.c "
        "os/hal/ports/STM32/LLD/ADCv2/adc_lld.c     "
        "os/hal/ports/STM32/LLD/CANv1/can_lld.c     "
        "os/hal/ports/STM32/LLD/EXTIv1/ext_lld.c    "
        "os/hal/ports/STM32/LLD/DACv1/dac_lld.c     "
        "os/hal/ports/STM32/LLD/DMAv2/stm32_dma.c   "
        "os/hal/ports/STM32/LLD/GPIOv2/pal_lld.c    "
        "os/hal/ports/STM32/LLD/I2Cv1/i2c_lld.c     "
        "os/hal/ports/STM32/LLD/MACv1/mac_lld.c     "
        "os/hal/ports/STM32/LLD/OTGv1/usb_lld.c     "
        "os/hal/ports/STM32/LLD/RTCv2/rtc_lld.c     "
        "os/hal/ports/STM32/LLD/SDIOv1/sdc_lld.c    "
        "os/hal/ports/STM32/LLD/SPIv1/i2s_lld.c     "
        "os/hal/ports/STM32/LLD/SPIv1/spi_lld.c     "
        "os/hal/ports/STM32/LLD/TIMv1/gpt_lld.c     "
        "os/hal/ports/STM32/LLD/TIMv1/icu_lld.c     "
        "os/hal/ports/STM32/LLD/TIMv1/pwm_lld.c     "
        "os/hal/ports/STM32/LLD/TIMv1/st_lld.c      "
        "os/hal/ports/STM32/LLD/USARTv1/serial_lld.c"
        "os/hal/ports/STM32/LLD/USARTv1/uart_lld.c  "
        "os/hal/ports/STM32/LLD/xWDGv1/wdg_lld.c    "
        )
include_directories(
        os/hal/ports/common/ARMCMx
        os/hal/ports/STM32/STM32F4xx
        os/hal/ports/STM32/LLD/ADCv2
        os/hal/ports/STM32/LLD/CANv1
        os/hal/ports/STM32/LLD/DACv1
        os/hal/ports/STM32/LLD/DMAv2
        os/hal/ports/STM32/LLD/EXTIv1
        os/hal/ports/STM32/LLD/GPIOv2
        os/hal/ports/STM32/LLD/I2Cv1
        os/hal/ports/STM32/LLD/MACv1
        os/hal/ports/STM32/LLD/OTGv1
        os/hal/ports/STM32/LLD/RTCv2
        os/hal/ports/STM32/LLD/SDIOv1
        os/hal/ports/STM32/LLD/SPIv1
        os/hal/ports/STM32/LLD/TIMv1
        os/hal/ports/STM32/LLD/USARTv1
        os/hal/ports/STM32/LLD/xWDGv1
)

# include $(CHIBIOS)/hw/board.mk
file(GLOB_RECURSE BOARD_SRC "hw/board.c")
include_directories(hw)

# include $(CHIBIOS)/os/hal/osal/rt/osal.mk
file(GLOB_RECURSE OSAL_SRC "os/hal/osal/rt/osal.c")
include_directories(os/hal/osal/rt)

# include $(CHIBIOS)/os/rt/rt.mk
file(GLOB_RECURSE RTOS_SRC "os/rt/src/*.c")
include_directories(os/rt/include)

# include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
file(GLOB_RECURSE PORT_SRC "os/rt/ports/ARMCMx/chcore.c" "os/rt/ports/ARMCMx/chcore_v7m.c")
include_directories(os/rt/ports/ARMCMx os/rt/ports/ARMCMx/compilers/GCC)

# include Shell
file(GLOB_RECURSE SHELL_SRC "os/various/shell.c" "os/various/evtimer.c" "os/various/syscalls.c"
        "os/hal/lib/streams/memstreams.c" "os/hal/lib/streams/chprintf.c")
include_directories(os/various os/hal/lib/streams)

add_library(USER ${USER_SRC})
add_library(STARTUP ${STARTUP_SRC} os/common/ports/ARMCMx/compilers/GCC/crt0_v7m.s)
add_library(HAL ${HAL_SRC})
add_library(PLATFORM ${PLATFORM_SRC})
add_library(BOARD ${BOARD_SRC})
add_library(OSAL ${OSAL_SRC})
add_library(RTOS ${RTOS_SRC})
add_library(PORT ${PORT_SRC} os/rt/ports/ARMCMx/compilers/GCC/chcoreasm_v7m.s)
add_library(SHELL ${SHELL_SRC})

# TODO: compile with CMakefile.txt
#add_definitions(-DSTM32F427xx)
#add_executable(${PROJECT_NAME}.elf ${USER_SRC} ${LINKER_SCRIPT})
#target_link_libraries(${PROJECT_NAME}.elf STARTUP HAL PLATFORM BOARD OSAL RTOS PORT)

#SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,-Map=binary.map -T ${LINKER_SCRIPT}")