set(target bike_app)

add_library(${target} STATIC)
set_target_properties(${target} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${out_app_lib_dir})
target_compile_definitions(${target} PRIVATE OSI_LOG_TAG=LOG_TAG_QUEC)
target_include_directories(${target} PUBLIC port hal_drv/inc common/inc app/inc ./)
set(APP_FILES
    app/src/ble_control.c
    app/src/ble_protocol.c
    app/src/app_system.c
    app/src/car_control.c)

set(COMMON_FILES 
    common/src/common.c
    common/src/ringbuffer.c)
    
set(HAL_DRV_FILES 
    hal_drv/src/hal_drv_gpio.c
    hal_drv/src/hal_drv_rtc.c
    hal_drv/src/hal_drv_uart.c
    hal_drv/src/hal_drv_flash.c)

target_sources(${target} PRIVATE
    bike_app_main.c
	${APP_FILES}
    ${COMMON_FILES}
    ${HAL_DRV_FILES}
    )

relative_glob(srcs include/*.h src/*.c inc/*.h)
beautify_c_code(${target} ${srcs})

