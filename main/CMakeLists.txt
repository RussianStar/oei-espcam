idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp32-camera driver esp_psram vfs esp_timer)

if(DEFINED SNAP_FRAME_SIZE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SNAP_FRAME_SIZE=${SNAP_FRAME_SIZE})
endif()

if(DEFINED SNAP_JPEG_QUALITY)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SNAP_JPEG_QUALITY=${SNAP_JPEG_QUALITY})
endif()

if(DEFINED SNAP_IDLE_DEINIT_MS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SNAP_IDLE_DEINIT_MS=${SNAP_IDLE_DEINIT_MS})
endif()

if(DEFINED SNAP_DEINIT)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SNAP_DEINIT=${SNAP_DEINIT})
endif()

if(DEFINED SNAP_SLEEP_AFTER_SNAP)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SNAP_SLEEP_AFTER_SNAP=${SNAP_SLEEP_AFTER_SNAP})
endif()

if(DEFINED SNAP_SLEEP_DELAY_MS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SNAP_SLEEP_DELAY_MS=${SNAP_SLEEP_DELAY_MS})
endif()

if(DEFINED SNAP_SELF_TEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SNAP_SELF_TEST=${SNAP_SELF_TEST})
endif()

if(DEFINED SNAP_I2C_PORT)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SNAP_I2C_PORT=${SNAP_I2C_PORT})
endif()

if(DEFINED SNAP_I2C_PORT_FALLBACK)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SNAP_I2C_PORT_FALLBACK=${SNAP_I2C_PORT_FALLBACK})
endif()
