set(LV_UI_DIR ui)
file(GLOB_RECURSE LV_UI_SOURCES ${LV_UI_DIR}/*.c)

set(srcs
    main.c
    speech_commands_action.c
    )

set(requires
    hardware_driver
    )

idf_component_register(SRCS ${srcs} ${LV_UI_SOURCES}
                       INCLUDE_DIRS include ${LV_UI_DIR}
                       REQUIRES ${requires} esp32_p4_box)

component_compile_options(-w)
