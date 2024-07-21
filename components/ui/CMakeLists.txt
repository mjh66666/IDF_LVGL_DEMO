# SET(SOURCES screens/ui_Screen1.c
#     ui.c
#     ui_helpers.c
#     ui_events.c
#     images/ui_img_1625496403.c)

# add_library(ui ${SOURCES})

idf_component_register(SRCS #源文件列表
    "screens/ui_Screen1.c"
    "ui.c"
    "ui_helpers.c"
    "images/ui_img_1625496403.c"
    INCLUDE_DIRS "."#目录列表
    REQUIRES "lvgl")#依赖的组件


