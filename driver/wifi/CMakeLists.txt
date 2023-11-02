message("Running makefsdata python script") 
execute_process(COMMAND 
    py makefsdata.py 
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} 
) 
 
add_executable(project 
    main.c 
) 
 
target_include_directories(project PRIVATE 
        ${CMAKE_CURRENT_LIST_DIR} 
        ) 
 
target_link_libraries(project 
    pico_cyw43_arch_lwip_threadsafe_background 
    pico_lwip_http 
    pico_stdlib 
    hardware_adc 
) 
 
pico_enable_stdio_usb(project TRUE) 
pico_enable_stdio_uart(project FALSE) 
 
pico_add_extra_outputs(project)