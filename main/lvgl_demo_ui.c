/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/widgets/extra/meter.html#simple-meter

#include "lvgl.h"

lv_obj_t *label_wifi;

void test_ui(void)
{
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x080808), 0);
    lv_label_set_text(label_wifi, "正在连接WiFi");
}
