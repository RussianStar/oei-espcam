idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp32-camera esp_driver_i2c esp_psram)

if(DEFINED SNAP_FRAME_SIZE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SNAP_FRAME_SIZE=${SNAP_FRAME_SIZE})
endif()

if(DEFINED SNAP_JPEG_QUALITY)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SNAP_JPEG_QUALITY=${SNAP_JPEG_QUALITY})
endif()
