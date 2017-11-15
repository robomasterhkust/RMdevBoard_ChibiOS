set(CMAKE_CXX_STANDARD 11)

cmake_minimum_required(VERSION 3.8)
project(RMdevBoard_ChibiOS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_custom_target(STLink COMMAND make upload -C ${RMdevBoard_ChibiOS_SOURCE_DIR}/dev
        CLION_EXE_DIR=${PROJECT_BINARY_DIR})

add_custom_target(Ozone COMMAND make -C ${RMdevBoard_ChibiOS_SOURCE_DIR}/dev
        CLION_EXE_DIR=${PROJECT_BINARY_DIR})
