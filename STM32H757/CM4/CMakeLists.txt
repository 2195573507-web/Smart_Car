cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(STM32_MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(STM32_LINKER_SCRIPT stm32h757xx_flash_CM4.ld)
set(STM32_LINKER_OPTION "")
set(CMAKE_PROJECT_NAME Smart_Car_H757_CM4)

project(${CMAKE_PROJECT_NAME})
add_executable(${CMAKE_PROJECT_NAME})
include("mx-generated.cmake")

set(CMAKE_CXX_COMPILER_FORCED "true")
enable_language(C ASM)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
