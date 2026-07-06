cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RM_Engineer_26_V1.0)

include(cmake/gcc-arm-none-eabi.cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        boards/bsp_rc.c
        boards/bsp_rc.h
        applications/remote_control.c
        applications/remote_control.h
        boards/bsp_usart.c
        boards/bsp_usart.h
        boards/bsp_can.c
        boards/bsp_can.h
        applications/cybergear.c
        applications/cybergear.h
        applications/LKMF9025.c
        applications/LKMF9025.h
        applications/daemon.c
        applications/daemon.h
        boards/bsp_buzzer.c
        boards/bsp_buzzer.h
        applications/Supercapacitor.c
        applications/Supercapacitor.h
        applications/CAN_receive.c
        applications/CAN_receive.h
        Tasks/chassis_control_task.c
        Tasks/chassis_control_task.h
        algorithm/pid.c
        algorithm/pid.h
        applications/DJI_Motor.c
        applications/DJI_Motor.h
        applications/DM_Motor.c
        applications/DM_Motor.h
        Tasks/shoot_task.c
        Tasks/shoot_task.h
        Tasks/rdk_comm_task.c
        Tasks/rdk_comm_task.h
        protocol/usart_protocol.c
        protocol/usart_protocol.h
        Tasks/com_with_gimbal.c
        Tasks/com_with_gimbal.h
        Info/robot_global.c
        Info/robot_global.h
        modules/can_motor.c
        modules/can_motor.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        algorithm
        applications
        boards
        Tasks
        protocol
        Info
        modules
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
#list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
