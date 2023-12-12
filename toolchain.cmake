set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
SET(CMAKE_CROSSCOMPILING 1)

set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_C_COMPILER_WORKS 1)



# set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)


set(CMAKE_CXX_FLAGS "-mcpu=cortex-m3 -mthumb -nostdlib -fno-exceptions")
set(CMAKE_C_FLAGS "-mcpu=cortex-m3 -mthumb -nostdlib")

# add_compile_options(-fexceptions)