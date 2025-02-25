//
// vim: ts=4 et
// Copyright (c) 2025 Petr Vanek, petr@fotoventus.cz
//
/// @file   main.cpp
/// @author Petr Vanek


#include "display_driver.h"



DisplayDriver display(DisplayDriver::DisplayRotation::Landscape_270);

extern "C" void app_main() {
    
    display.initBus();
    display.backLight(10);

    display.start();

    display.lock();
    
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(0, 0, 255), 0);
    lv_obj_t *rect = lv_obj_create(lv_scr_act()); 
    lv_obj_set_size(rect, 320, 172);  
    lv_obj_set_style_bg_color(rect, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_align(rect, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *label = lv_label_create(rect);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);  
    lv_label_set_text(label, "01234567");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);  

    display.unlock(); 

}
