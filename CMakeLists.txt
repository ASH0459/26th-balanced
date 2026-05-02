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

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# Set the project name
set(CMAKE_PROJECT_NAME 26th_balance_infantry)

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
        Users/Algorithm/Alg_EKF/EKF/QuaternionEKF.c
        Users/Algorithm/Alg_EKF/EKF/QuaternionEKF.h
        Users/Algorithm/Alg_EKF/kalman/kalman_filter.c
        Users/Algorithm/Alg_EKF/kalman/kalman_filter.h
        Users/Algorithm/Alg_EKF/mahony/mahony_filter.c
        Users/Algorithm/Alg_EKF/mahony/mahony_filter.h
        Users/Algorithm/Alg_Filter/Alg_Filter.c
        Users/Algorithm/Alg_Filter/Alg_Filter.h
        Users/Algorithm/Alg_PID/Alg_PID.c
        Users/Algorithm/Alg_PID/Alg_PID.h
        Users/Algorithm/Alg_RLS/Alg_RLS.hpp
        Users/Algorithm/Alg_RLS/Matrix.hpp
        Users/Algorithm/Alg_Support/Alg_UserLib.c
        Users/Algorithm/Alg_Support/Alg_UserLib.h
        Users/Application/APP_Buzzer/APP_Buzzer.cpp
        Users/Application/APP_Buzzer/APP_Buzzer.h
        Users/Application/APP_Chassis/App_Chassis_Behaviour.cpp
        Users/Application/APP_Chassis/App_Chassis_Behaviour.h
        Users/Application/APP_Chassis/App_Chassis_Task.cpp
        Users/Application/APP_Chassis/App_Chassis_Task.h
        Users/Application/APP_Chassis/App_Chassis_Task_Helpers.cpp
        Users/Application/APP_Chassis/App_Chassis_Task_Helpers.h
        Users/Application/App_Detect/App_Detect_Task.c
        Users/Application/App_Detect/App_Detect_Task.h
        Users/Application/APP_INS/INS_task.c
        Users/Application/APP_INS/INS_task.h
        Users/Application/APP_LED/led_flow_task.c
        Users/Application/APP_LED/led_flow_task.h
        Users/Bsp/Bsp_Boards/Board_Buzzer/bsp_buzzer.c
        Users/Bsp/Bsp_Boards/Board_Buzzer/bsp_buzzer.h
        Users/Bsp/Bsp_Boards/Board_CAN/bsp_can.c
        Users/Bsp/Bsp_Boards/Board_CAN/bsp_can.h
        Users/Bsp/Bsp_Boards/Board_CRC/Board_CRC8_CRC16.c
        Users/Bsp/Bsp_Boards/Board_CRC/Board_CRC8_CRC16.h
        Users/Bsp/Bsp_Boards/Board_CRC/Board_CRC32.c
        Users/Bsp/Bsp_Boards/Board_CRC/Board_CRC32.h
        Users/Bsp/Bsp_Boards/Board_Dwt/bsp_dwt.c
        Users/Bsp/Bsp_Boards/Board_Dwt/bsp_dwt.h
        Users/Bsp/Bsp_Boards/Board_IMU_Pwm/bsp_imu_pwm.c
        Users/Bsp/Bsp_Boards/Board_IMU_Pwm/bsp_imu_pwm.h
        Users/Bsp/Bsp_Boards/Bsp_Led/bsp_led.c
        Users/Bsp/Bsp_Boards/Bsp_Led/bsp_led.h
        Users/Bsp/Bsp_Device/Dev_BMI088_Driver/BMI088driver.c
        Users/Bsp/Bsp_Device/Dev_BMI088_Driver/BMI088driver.h
        Users/Bsp/Bsp_Device/Dev_BMI088_Driver/BMI088Middleware.c
        Users/Bsp/Bsp_Device/Dev_BMI088_Driver/BMI088Middleware.h
        Users/Bsp/Bsp_Device/Dev_BMI088_Driver/BMI088reg.h
        Users/Bsp/Bsp_Device/Dev_Can_Receive/Dev_Can_Receive.cpp
        Users/Bsp/Bsp_Device/Dev_Can_Receive/Dev_Can_Receive.h
        Users/Upper_Communication/UC_Referee/UC_Protocol.c
        Users/Upper_Communication/UC_Referee/UC_Protocol.h
        Users/Upper_Communication/UC_Referee/UC_Referee_Protocol_Contract.c
        Users/Upper_Communication/UC_Referee/UC_Referee.c
        Users/Upper_Communication/UC_Referee/UC_Referee.h
        Middlewares/ST/ARM/DSP/Inc/arm_math.h
        Users/Bsp/Bsp_Support/Sp_CRC8_CRC16/Sp_CRC8_CRC16.c
        Users/Bsp/Bsp_Support/Sp_CRC8_CRC16/Sp_CRC8_CRC16.h
        Users/Bsp/Bsp_Support/Sp_FIFO/Sp_FIFO.c
        Users/Bsp/Bsp_Support/Sp_FIFO/Sp_FIFO.h
        Users/Bsp/Bsp_Support/Sp_MEM_MANG/Sp_MEM_MANG.c
        Users/Bsp/Bsp_Support/Sp_MEM_MANG/Sp_MEM_MANG.h
        Users/Bsp/Bsp_Support/linux_list.h
        Users/Bsp/Bsp_Support/macro_mutex.h
        Users/Bsp/Bsp_Boards/Board_USART/Board_USART.c
        Users/Bsp/Bsp_Boards/Board_USART/Board_USART.h
        Users/Application/App_Referee/App_Referee_Task.c
        Users/Bsp/Bsp_Device/Dev_Referee/Dev_Referee.c
        Users/Algorithm/Alg_wbr/wbr.c
        Users/0-APL/APL_PowerLimitator/APL_PowerLimitator.cpp
        Users/0-APL/APL_PowerLimitator/APL_PowerLimitator.hpp
        Users/0-APL/APL_SuperCap/APL_SuperCap.cpp
        Users/0-APL/APL_SuperCap/APL_SuperCap.hpp
        Users/2-HDL/HDL_PowerLimitator/HDL_PowerLimitator.cpp
        Users/2-HDL/HDL_PowerLimitator/HDL_PowerLimitator.hpp
        Users/2-HDL/HDL_SuperCap/HDL_SuperCap.cpp
        Users/2-HDL/HDL_SuperCap/HDL_SuperCap.hpp
        Users/Application/App_UI/UI_Frames/ui_start.c
        Users/Application/App_UI/UI_Frames/ui_start.h
        Users/Application/App_UI/UI_Frames/ui_normal.c
        Users/Application/App_UI/UI_Frames/ui_normal.h
        Users/Application/App_UI/App_UI_Task.cpp
        Users/Application/App_UI/App_UI_Task.h
        Users/Application/App_UI/ui.h
        Users/Application/App_UI/ui_interface.c
        Users/Application/App_UI/ui_interface.h
        Users/Application/App_UI/ui_types.h)

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
    # Add user defined include paths
)

include_directories(Core/Inc Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/CMSIS/Include Middlewares/ST/ARM/DSP/Inc)
include_directories(Users/Application/Inc Users/Bsp/Bsp_Boards/Inc Components/Algorithm/Inc Components/Devices/Inc Users/Algorithm/Alg_EKF/EKF Users/Algorithm/Alg_Filter Users/Algorithm/Alg_PID Users/Algorithm/Alg_RLS Users/Algorithm/Alg_Support Users/Algorithm/Alg_VMC)
include_directories(Users/Bsp/Bsp_Device/Dev_Can_Receive Users/Bsp/Bsp_Device/Dev_BMI088_Driver)
include_directories(Users/Application/App_Detect Users/Application/APP_Chassis Users/Algorithm/Alg_EKF/mahony)
include_directories(Drivers/CMSIS/Device/ST/STM32H7xx/Include Users/Upper_Communication/UC_Referee Users/Bsp/Bsp_Device/Dev_Custom Users/Bsp/Bsp_Boards/Board_CRC)
include_directories(Users/Bsp/Bsp_Boards/Board_Buzzer Users/Bsp/Bsp_Boards/Board_CAN Users/Bsp/Bsp_Boards/Board_IMU_Pwm)
include_directories(Users/Bsp/Bsp_Boards/Bsp_Led Users/Application/APP_INS Users/Application/APP_LED Users/Application/APP_Buzzer Users/Bsp/Bsp_Boards/Board_Dwt)
include_directories(Core/Inc Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/CMSIS/Include DSP_LIB/Include)
include_directories(Middlewares/ST/ARM/DSP/Inc Users/Bsp/Bsp_Support/Sp_CRC8_CRC16 Users/Bsp/Bsp_Support/Sp_FIFO Users/Bsp/Bsp_Support/Sp_MEM_MANG Users/Bsp/Bsp_Support/)
include_directories(Users/Bsp/Bsp_Device/Dev_Referee Users/Application/App_Referee Users/Bsp/Bsp_Boards/Board_USART Users/Algorithm/Alg_wbr  Users/Algorithm/Alg_EKF/kalman)
include_directories(Users/0-APL/APL_SuperCap Users/0-APL/APL_PowerLimitator Users/2-HDL/HDL_PowerLimitator Users/2-HDL/HDL_SuperCap)
include_directories(Users/Application/App_UI Users/Application/App_UI/UI_Frames)
# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

target_link_directories(stm32cubemx INTERFACE
        ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/Lib
)

target_link_libraries(stm32cubemx INTERFACE
        #        :libarm_cortexM7l_math.a
        #:libarm_cortexM7lfdp_math.a
        :libarm_cortexM7lfsp_math.a
)
