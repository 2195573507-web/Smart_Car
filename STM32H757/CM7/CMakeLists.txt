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

# Enable floating-point formatting in newlib-nano for DUAL_AHRS diagnostics.
target_link_options(${PROJECT_NAME} PRIVATE -u _printf_float)

# The normal application owns both IMU acquisition paths. Set this option ON
# only for the bounded BMI323 diagnostic image.
option(SMARTCAR_BMI323_DEBUG_ONLY
       "Run the CM7 BMI323-only diagnostic configuration" OFF)

# Project-owned sources. CubeMX IOC and generated source lists remain unchanged.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/IMU/LSM303/lsm303.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Sensor/BMI323/bmi323.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Sensor/BMI323/bmi323_port.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../BSP/TIMER/imu_time.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Sensor/imu_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Calibration/imu_calibration.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Calibration/imu_vibration.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Calibration/imu_boot_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Filter/imu_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Sensor/Magnetometer/mag_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Attitude/attitude.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Attitude/Leveling/imu_leveling.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Attitude/DualAHRS/dual_ahrs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Boot/boot_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Application/RTOS/imu_runtime.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Communication/UART_Link/uart_link.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Communication/SmartCar_Frame/sc_frame.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Communication/Services/s3_service.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Communication/Services/log_service.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../System/Task/rtos_health.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/portable/MemMang/heap_4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/portable/GCC/ARM_CM7/r0p1/port.c
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Communication/UART_Link
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Communication/SmartCar_Frame
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Communication/Services
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/IMU/BMI323
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/IMU/LSM303
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Calibration
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Filter
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Sensor/Magnetometer
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Attitude
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Attitude/Leveling
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Attitude/DualAHRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/Boot
    ${CMAKE_CURRENT_SOURCE_DIR}/../Application/RTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/../System/Task
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Middleware/FreeRTOS/portable/GCC/ARM_CM7/r0p1
    ${CMAKE_CURRENT_SOURCE_DIR}/../Config
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    IMU_MANAGER_USE_FREERTOS=1
    SMARTCAR_BMI323_DEBUG_ONLY=$<BOOL:${SMARTCAR_BMI323_DEBUG_ONLY}>
)

set(CMAKE_CXX_COMPILER_FORCED "true")
enable_language(C ASM)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
