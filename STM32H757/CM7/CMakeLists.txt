cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(STM32_MCU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")
set(STM32_LINKER_SCRIPT stm32h757xx_flash_CM7.ld)
set(STM32_LINKER_OPTION "")
set(CMAKE_PROJECT_NAME Smart_Car_H757_CM7)

project(${CMAKE_PROJECT_NAME})
add_executable(${CMAKE_PROJECT_NAME})
include("mx-generated.cmake")

# Project-owned IMU sources. CubeMX IOC and generated source lists remain unchanged.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/IMU/BMI323/bmi323.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/IMU/LSM303/lsm303.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Sensor/imu_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Calibration/imu_calibration.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Filter/imu_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Attitude/attitude.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Boot/boot_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Application/RTOS/imu_runtime.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/portable/MemMang/heap_4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/portable/GCC/ARM_CM7/r0p1/port.c
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/IMU/BMI323
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/IMU/LSM303
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Calibration
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Filter
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Attitude
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Boot
    ${CMAKE_CURRENT_SOURCE_DIR}/../Application/RTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/portable/GCC/ARM_CM7/r0p1
    ${CMAKE_CURRENT_SOURCE_DIR}/../Config
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE IMU_MANAGER_USE_FREERTOS=1)

set(CMAKE_CXX_COMPILER_FORCED "true")
enable_language(C ASM)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
