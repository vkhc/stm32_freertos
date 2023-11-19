
set(exec ${CMAKE_PROJECT_NAME}_main)


add_executable(${exec}
    main.cpp
)

target_compile_options(${exec} PRIVATE -Wvolatile)

target_link_options(${exec} PUBLIC -T${linkerFile})


target_link_libraries(${exec} PRIVATE
    startup
    vendor
    freertos_kernel freertos_config
)

set_target_properties(${exec} 
    PROPERTIES
        OUTPUT_NAME  "blink"
        SUFFIX       ".elf"
)