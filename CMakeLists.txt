set(target bike_app)


add_library(${target} STATIC)
set_target_properties(${target} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${out_app_lib_dir})
target_compile_definitions(${target} PRIVATE OSI_LOG_TAG=LOG_TAG_QUEC)
target_include_directories(${target} PUBLIC port hal_drv/inc common/inc app/inc ./ )
set(APP_FILES
    app/src/ble_control.c
    app/src/ble_protocol.c
    app/src/app_system.c
    app/src/car_control.c
    app/src/app_rtc_task.c
    app/src/app_virt_uart.c
    app/src/app_led_ind.c
    app/src/net_protocol.c
    app/src/net_control.c
    app/src/mcu_uart.c
    app/src/can_protocol.c
    app/src/net_control.c
    app/src/lock.c
    app/src/http_upgrade_ota.c
    app/src/qmi8658.c
    app/src/app_sensor.c
    app/src/GPS_control.c
    app/src/app_audio.c
    app/src/low_power.c
    app/src/net_engwe_protocol.c
    app/src/app_error.c
 )

set(COMMON_FILES 
    common/src/app_common.c
    common/src/ringbuffer.c)


set(HAL_DRV_FILES 
    hal_drv/src/hal_drv_gpio.c
    hal_drv/src/hal_drv_rtc.c
    hal_drv/src/hal_drv_uart.c
    hal_drv/src/hal_drv_flash.c
    hal_drv/src/hal_virt_at.c
    hal_drv/src/hal_drv_net.c
    hal_drv/src/hal_drv_iic.c
    )

target_sources(${target} PRIVATE
    bike_app_main.c
	${APP_FILES}
    ${COMMON_FILES}
    ${HAL_DRV_FILES}
    )
#add_library(mylib STATIC mylib.c)
#target_link_libraries(mylib m c)
find_library(MYLIB_LIBRARY imualgo PATHS "${CMAKE_CURRENT_SOURCE_DIR}/app/src")
target_link_libraries(${target} ${MYLIB_LIBRARY})

# find_library(MYLIB_LIBRARYt mylib PATHS "${CMAKE_CURRENT_SOURCE_DIR}/app/src")
# target_link_libraries(${target} ${MYLIB_LIBRARYt})
target_link_libraries(${target} m c)
relative_glob(srcs include/*.h src/*.c inc/*.h)
beautify_c_code(${target} ${srcs})

