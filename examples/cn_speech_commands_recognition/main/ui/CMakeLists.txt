SET(SOURCES screens/ui_ScreenSR.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_1924961647.c
    images/ui_img_1545807356.c
    images/ui_img_1126967247.c
    images/ui_img_1706561437.c
    images/ui_img_424947501.c
    images/ui_img_1184979050.c
    images/ui_img_coffee_pic_2_png.c
    images/ui_img_374834912.c
    images/ui_img_1283678341.c
    fonts/ui_font_FontPromt.c
    fonts/ui_font_FontSR.c
    fonts/ui_font_FontSR36.c)

add_library(ui ${SOURCES})
