if (EXISTS ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c)
    add_executable(car_nosys
            car.c
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
            )
    target_compile_definitions(car_nosys PRIVATE
            WIFI_SSID=\"${WIFI_SSID}\"
            WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
            )
    target_include_directories(car_nosys PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping
            )
    target_link_libraries(car_nosys
                pico_cyw43_arch_lwip_threadsafe_background
                pico_stdlib
                pico_lwip_iperf
                FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
            )
        pico_add_extra_outputs(car_nosys)

    add_executable(car_sys
            car.c
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
            )
    target_compile_definitions(car_sys PRIVATE
            #WIFI_SSID=\"${WIFI_SSID}\"
            #WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
            NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
            LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
            PING_USE_SOCKETS=1
            )
    target_include_directories(car_sys PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping
            )
    target_link_libraries(car_sys
            hardware_adc
            hardware_i2c
            hardware_timer
            hardware_gpio
            pico_cyw43_arch_lwip_sys_freertos
            hardware_pwm
            pico_stdlib
            pico_lwip_iperf
            FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap 
            pico_lwip_http 
            hardware_irq
            )
    pico_enable_stdio_usb(car_sys 1)
    pico_enable_stdio_uart(car_sys 1)
    pico_add_extra_outputs(car_sys)
    example_auto_set_url(car_sys)

endif()