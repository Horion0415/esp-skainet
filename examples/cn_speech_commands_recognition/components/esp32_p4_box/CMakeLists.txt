#IDF version is less than IDF5.0
# if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
if(0)
    set(SRC_VER "esp-box-3_idf4.c")
else()
    set(SRC_VER "esp32_p4_box_idf5.c")
endif()

idf_component_register(
    SRCS "esp32_p4_box.c" ${SRC_VER}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver hardware_driver
    PRIV_REQUIRES esp_lcd usb spiffs fatfs
)
