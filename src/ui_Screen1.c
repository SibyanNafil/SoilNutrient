// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 9.1.0
// Project name: Soil_Nutrient_try2

#include "ui.h"

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_remove_flag(ui_Screen1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_SCROLLABLE |
                       LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM);     /// Flags
    lv_obj_set_scrollbar_mode(ui_Screen1, LV_SCROLLBAR_MODE_OFF);

    ui_RootPanel = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_RootPanel, lv_pct(100));
    lv_obj_set_height(ui_RootPanel, lv_pct(100));
    lv_obj_set_align(ui_RootPanel, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_RootPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_RootPanel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_RootPanel, lv_color_hex(0x13F54C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_RootPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TopPanel = lv_obj_create(ui_RootPanel);
    lv_obj_set_height(ui_TopPanel, 30);
    lv_obj_set_width(ui_TopPanel, lv_pct(111));
    lv_obj_set_x(ui_TopPanel, 2);
    lv_obj_set_y(ui_TopPanel, -103);
    lv_obj_set_align(ui_TopPanel, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_TopPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_TopPanel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_TopPanel, lv_color_hex(0x0C5F2A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_TopPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Title = lv_label_create(ui_TopPanel);
    lv_obj_set_width(ui_Title, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Title, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Title, -3);
    lv_obj_set_y(ui_Title, -1);
    lv_obj_set_align(ui_Title, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Title, "SOIL NUTRIENTS");
    lv_obj_set_style_text_font(ui_Title, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_AllValuePanel = lv_obj_create(ui_RootPanel);
    lv_obj_set_width(ui_AllValuePanel, 148);
    lv_obj_set_height(ui_AllValuePanel, 199);
    lv_obj_set_x(ui_AllValuePanel, 8);
    lv_obj_set_y(ui_AllValuePanel, 21);
    lv_obj_set_align(ui_AllValuePanel, LV_ALIGN_TOP_RIGHT);
    lv_obj_remove_flag(ui_AllValuePanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_AllValuePanel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TempPanel = lv_obj_create(ui_AllValuePanel);
    lv_obj_set_width(ui_TempPanel, 131);
    lv_obj_set_height(ui_TempPanel, 27);
    lv_obj_set_x(ui_TempPanel, -1);
    lv_obj_set_y(ui_TempPanel, -7);
    lv_obj_set_align(ui_TempPanel, LV_ALIGN_TOP_MID);
    lv_obj_remove_flag(ui_TempPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_TempPanel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_TempPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_KetTemp = lv_label_create(ui_TempPanel);
    lv_obj_set_width(ui_KetTemp, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_KetTemp, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_KetTemp, -37);
    lv_obj_set_y(ui_KetTemp, -1);
    lv_obj_set_align(ui_KetTemp, LV_ALIGN_CENTER);
    lv_label_set_text(ui_KetTemp, "Temp =");
    lv_obj_set_style_text_font(ui_KetTemp, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TempValueLabel = lv_label_create(ui_TempPanel);
    lv_obj_set_width(ui_TempValueLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_TempValueLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_TempValueLabel, 11);
    lv_obj_set_y(ui_TempValueLabel, 0);
    lv_obj_set_align(ui_TempValueLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_TempValueLabel, "0.00");

    ui_UnitTemp = lv_label_create(ui_TempPanel);
    lv_obj_set_width(ui_UnitTemp, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_UnitTemp, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_UnitTemp, 50);
    lv_obj_set_y(ui_UnitTemp, 0);
    lv_obj_set_align(ui_UnitTemp, LV_ALIGN_CENTER);
    lv_label_set_text(ui_UnitTemp, "°C");

    ui_HumPanel = lv_obj_create(ui_AllValuePanel);
    lv_obj_set_width(ui_HumPanel, 131);
    lv_obj_set_height(ui_HumPanel, 27);
    lv_obj_set_x(ui_HumPanel, 0);
    lv_obj_set_y(ui_HumPanel, -46);
    lv_obj_set_align(ui_HumPanel, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_HumPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_HumPanel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_HumPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_KetHum = lv_label_create(ui_HumPanel);
    lv_obj_set_width(ui_KetHum, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_KetHum, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_KetHum, -37);
    lv_obj_set_y(ui_KetHum, -1);
    lv_obj_set_align(ui_KetHum, LV_ALIGN_CENTER);
    lv_label_set_text(ui_KetHum, "Hum =");
    lv_obj_set_style_text_font(ui_KetHum, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_HumValueLabel = lv_label_create(ui_HumPanel);
    lv_obj_set_width(ui_HumValueLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_HumValueLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_HumValueLabel, 11);
    lv_obj_set_y(ui_HumValueLabel, 0);
    lv_obj_set_align(ui_HumValueLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_HumValueLabel, "0");
    lv_obj_set_style_text_font(ui_HumValueLabel, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_UnitHum = lv_label_create(ui_HumPanel);
    lv_obj_set_width(ui_UnitHum, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_UnitHum, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_UnitHum, 50);
    lv_obj_set_y(ui_UnitHum, 0);
    lv_obj_set_align(ui_UnitHum, LV_ALIGN_CENTER);
    lv_label_set_text(ui_UnitHum, "%");

    ui_PhPanel = lv_obj_create(ui_AllValuePanel);
    lv_obj_set_width(ui_PhPanel, 131);
    lv_obj_set_height(ui_PhPanel, 27);
    lv_obj_set_x(ui_PhPanel, 0);
    lv_obj_set_y(ui_PhPanel, -15);
    lv_obj_set_align(ui_PhPanel, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_PhPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_PhPanel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_PhPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_KetPh = lv_label_create(ui_PhPanel);
    lv_obj_set_width(ui_KetPh, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_KetPh, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_KetPh, -37);
    lv_obj_set_y(ui_KetPh, -1);
    lv_obj_set_align(ui_KetPh, LV_ALIGN_CENTER);
    lv_label_set_text(ui_KetPh, "PH     =");
    lv_obj_set_style_text_font(ui_KetPh, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_PhValueLabel = lv_label_create(ui_PhPanel);
    lv_obj_set_width(ui_PhValueLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_PhValueLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_PhValueLabel, 11);
    lv_obj_set_y(ui_PhValueLabel, 0);
    lv_obj_set_align(ui_PhValueLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_PhValueLabel, "0");
    lv_obj_set_style_text_font(ui_PhValueLabel, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_NPanel = lv_obj_create(ui_AllValuePanel);
    lv_obj_set_width(ui_NPanel, 131);
    lv_obj_set_height(ui_NPanel, 27);
    lv_obj_set_x(ui_NPanel, 0);
    lv_obj_set_y(ui_NPanel, 16);
    lv_obj_set_align(ui_NPanel, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_NPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_NPanel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_NPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_KetN = lv_label_create(ui_NPanel);
    lv_obj_set_width(ui_KetN, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_KetN, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_KetN, -37);
    lv_obj_set_y(ui_KetN, -1);
    lv_obj_set_align(ui_KetN, LV_ALIGN_CENTER);
    lv_label_set_text(ui_KetN, "N        =");
    lv_obj_set_style_text_font(ui_KetN, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_NValueLabel = lv_label_create(ui_NPanel);
    lv_obj_set_width(ui_NValueLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_NValueLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_NValueLabel, 11);
    lv_obj_set_y(ui_NValueLabel, 0);
    lv_obj_set_align(ui_NValueLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_NValueLabel, "0");
    lv_obj_set_style_text_font(ui_NValueLabel, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_UnitN = lv_label_create(ui_NPanel);
    lv_obj_set_width(ui_UnitN, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_UnitN, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_UnitN, 47);
    lv_obj_set_y(ui_UnitN, -1);
    lv_obj_set_align(ui_UnitN, LV_ALIGN_CENTER);
    lv_label_set_text(ui_UnitN, "mg\n     /kg");
    lv_obj_set_style_text_font(ui_UnitN, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_PPanel = lv_obj_create(ui_AllValuePanel);
    lv_obj_set_width(ui_PPanel, 131);
    lv_obj_set_height(ui_PPanel, 27);
    lv_obj_set_x(ui_PPanel, 0);
    lv_obj_set_y(ui_PPanel, 47);
    lv_obj_set_align(ui_PPanel, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_PPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_PPanel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_PPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_KetP = lv_label_create(ui_PPanel);
    lv_obj_set_width(ui_KetP, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_KetP, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_KetP, -36);
    lv_obj_set_y(ui_KetP, -1);
    lv_obj_set_align(ui_KetP, LV_ALIGN_CENTER);
    lv_label_set_text(ui_KetP, "P        =");
    lv_obj_set_style_text_font(ui_KetP, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_PValueLabel = lv_label_create(ui_PPanel);
    lv_obj_set_width(ui_PValueLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_PValueLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_PValueLabel, 11);
    lv_obj_set_y(ui_PValueLabel, 0);
    lv_obj_set_align(ui_PValueLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_PValueLabel, "0");
    lv_obj_set_style_text_font(ui_PValueLabel, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_UnitP = lv_label_create(ui_PPanel);
    lv_obj_set_width(ui_UnitP, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_UnitP, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_UnitP, 47);
    lv_obj_set_y(ui_UnitP, -1);
    lv_obj_set_align(ui_UnitP, LV_ALIGN_CENTER);
    lv_label_set_text(ui_UnitP, "mg\n     /kg");
    lv_obj_set_style_text_font(ui_UnitP, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_KPanel = lv_obj_create(ui_AllValuePanel);
    lv_obj_set_width(ui_KPanel, 131);
    lv_obj_set_height(ui_KPanel, 27);
    lv_obj_set_x(ui_KPanel, 0);
    lv_obj_set_y(ui_KPanel, 79);
    lv_obj_set_align(ui_KPanel, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_KPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_KPanel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_KPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_KetK = lv_label_create(ui_KPanel);
    lv_obj_set_width(ui_KetK, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_KetK, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_KetK, -36);
    lv_obj_set_y(ui_KetK, -1);
    lv_obj_set_align(ui_KetK, LV_ALIGN_CENTER);
    lv_label_set_text(ui_KetK, "K        =");
    lv_obj_set_style_text_font(ui_KetK, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_KValueLabel = lv_label_create(ui_KPanel);
    lv_obj_set_width(ui_KValueLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_KValueLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_KValueLabel, 11);
    lv_obj_set_y(ui_KValueLabel, 0);
    lv_obj_set_align(ui_KValueLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_KValueLabel, "0");
    lv_obj_set_style_text_font(ui_KValueLabel, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label4 = lv_label_create(ui_KPanel);
    lv_obj_set_width(ui_Label4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label4, 47);
    lv_obj_set_y(ui_Label4, -1);
    lv_obj_set_align(ui_Label4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label4, "mg\n     /kg");
    lv_obj_set_style_text_font(ui_Label4, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_InfoPanel = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_InfoPanel, 154);
    lv_obj_set_height(ui_InfoPanel, 200);
    lv_obj_set_x(ui_InfoPanel, -78);
    lv_obj_set_y(ui_InfoPanel, 16);
    lv_obj_set_align(ui_InfoPanel, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_InfoPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_InfoPanel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_InfoPanel, lv_color_hex(0x13F54C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_InfoPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Logo = lv_image_create(ui_InfoPanel);
    lv_image_set_src(ui_Logo, &ui_img_1509005985);
    lv_obj_set_width(ui_Logo, LV_SIZE_CONTENT);   /// 470
    lv_obj_set_height(ui_Logo, LV_SIZE_CONTENT);    /// 100
    lv_obj_set_x(ui_Logo, -168);
    lv_obj_set_y(ui_Logo, -31);
    lv_obj_set_align(ui_Logo, LV_ALIGN_LEFT_MID);
    lv_obj_add_flag(ui_Logo, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_remove_flag(ui_Logo, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_image_set_scale(ui_Logo, 70);
    lv_obj_set_style_bg_color(ui_Logo, lv_color_hex(0x0C5F2A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Logo, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_InfoLabel = lv_label_create(ui_InfoPanel);
    lv_obj_set_width(ui_InfoLabel, lv_pct(122));
    lv_obj_set_height(ui_InfoLabel, lv_pct(28));
    lv_obj_set_x(ui_InfoLabel, 1);
    lv_obj_set_y(ui_InfoLabel, 71);
    lv_obj_set_align(ui_InfoLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_InfoLabel, "IG : \n@pkmkc.soilnutriens\nEmail : \nsoilnutrienspkm@gmail.com");
    lv_obj_set_style_text_align(ui_InfoLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_InfoLabel, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_InfoLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_InfoLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_InfoLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

}
