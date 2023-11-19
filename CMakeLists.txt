include(${CMAKE_CURRENT_SOURCE_DIR}/toolchain.cmake)


project(stm32)

cmake_minimum_required(VERSION 3.20)
# set(CMAKE_VERBOSE_MAKEFILE on)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED true)

set(device STM32L152xE)
set(linkerFile ${PROJECT_SOURCE_DIR}/linker/linker.ld)


########### FREERTOS CONFIG MOVE SOMWHERE
set(FREERTOS_KERNEL_PATH "../../")

# Add the freertos_config for FreeRTOS-Kernel
add_library(freertos_config INTERFACE)

target_include_directories(freertos_config
    INTERFACE
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(freertos_config INTERFACE vendor)

# Select the heap port.  values between 1-4 will pick a heap.
set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)

# Select the native compile PORT
set(FREERTOS_PORT "GCC_ARM_CM3" CACHE STRING "" FORCE)

########### FREERTOS CONFIG MOVE SOMWHERE

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/startup)
add_subdirectory(${PROJECT_SOURCE_DIR}/vendor)
add_subdirectory(${PROJECT_SOURCE_DIR}/rtos/FreeRTOS-Kernel)


add_custom_target(flash
                  COMMAND openocd -f interface/stlink.cfg -f target/stm32l1.cfg -c "program src/blink.elf verify reset exit"
                  VERBATIM
                  USES_TERMINAL
)