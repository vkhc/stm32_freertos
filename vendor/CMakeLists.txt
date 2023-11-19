add_library(vendor
    CMSIS/Device/ST/STM32L1/Source/Templates/system_stm32l1xx.c
)

target_include_directories(vendor PUBLIC
    CMSIS/Device/ST/STM32L1/Include
    CMSIS/CMSIS/Core/Include
)

target_compile_definitions(vendor PUBLIC
        -D${device}
        )
