// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.6
// Project name: han-wt32

#include "../ui.h"

void ui_SplashScreen_screen_init(void)
{
    ui_SplashScreen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_SplashScreen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_img_src(ui_SplashScreen, &ui_img_sky_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label3 = lv_label_create(ui_SplashScreen);
    lv_obj_set_width(ui_Label3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label3, -84);
    lv_obj_set_y(ui_Label3, -128);
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "Han Solominer");
    lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0xFAFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label3, &ui_font_star32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label1 = lv_label_create(ui_SplashScreen);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, -72);
    lv_obj_set_y(ui_Label1, -93);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "The rise of hashrate");
    lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0xFAFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label1, &ui_font_star24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_lblpassword = lv_label_create(ui_SplashScreen);
    lv_obj_set_width(ui_lblpassword, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblpassword, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblpassword, 7);
    lv_obj_set_y(ui_lblpassword, 286);
    lv_label_set_text(ui_lblpassword, "password");
    lv_obj_set_style_text_color(ui_lblpassword, lv_color_hex(0xFAFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_lblpassword, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_lblpassword, &ui_font_start16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_lblssid = lv_label_create(ui_SplashScreen);
    lv_obj_set_width(ui_lblssid, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblssid, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblssid, 8);
    lv_obj_set_y(ui_lblssid, 259);
    lv_label_set_text(ui_lblssid, "ssid");
    lv_obj_set_style_text_color(ui_lblssid, lv_color_hex(0xFAFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_lblssid, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_lblssid, &ui_font_start16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_lblversion = lv_label_create(ui_SplashScreen);
    lv_obj_set_width(ui_lblversion, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblversion, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblversion, -5);
    lv_obj_set_y(ui_lblversion, 284);
    lv_obj_set_align(ui_lblversion, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(ui_lblversion, "version ");
    lv_obj_set_style_text_color(ui_lblversion, lv_color_hex(0xFAFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_lblversion, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_lblversion, &ui_font_start16, LV_PART_MAIN | LV_STATE_DEFAULT);

}
