//
// vim: ts=4 et
// Copyright (c) 2025 Petr Vanek, petr@fotoventus.cz
//
/// @file   main.cpp
/// @author Petr Vanek


#include "display_driver.h"
#include "sd_card.h"
#include "sd_card.h"
#include "rgb_led.h"
#include "esp_log.h"



void sdcardExample(SdCard& sdCard)
{
    static constexpr const char *TAG = "SD_Example";

    if (sdCard.mount(true) != ESP_OK)
    {
        ESP_LOGE(TAG, "SD card mount failed!");
        return;
    }
    ESP_LOGI(TAG, "SD card mounted successfully!");

    std::string filename = "/file.txt";
    std::string dataToWrite = "Hello from ESP32!";

    if (sdCard.writeFile(filename, dataToWrite))
    {
        ESP_LOGI(TAG, "File written successfully: %s", filename.c_str());
    }
    else
    {
        ESP_LOGE(TAG, "Failed to write file: %s", filename.c_str());
    }

    std::string readData = sdCard.readFile(filename);
    if (!readData.empty())
    {
        ESP_LOGI(TAG, "File read successfully: %s", readData.c_str());
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read file: %s", filename.c_str());
    }

    if (sdCard.unmount() == ESP_OK)
    {
        ESP_LOGI(TAG, "SD card unmounted successfully!");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to unmount SD card!");
    }
}


// display
DisplayDriver display(DisplayDriver::DisplayRotation::Landscape_270);

// init SPI (shared with LCD)
SdCard	sdcard("/sdcard", HW_SD_MOSI, HW_SD_MISO, HW_SD_CLK, HW_SD_CS);


RGBLed led(HW_LED_RGB); 

extern "C" void app_main() {

    sdcard.initBus(true);

    // disable init SPI in display
    display.initBus(false);
    display.backLight(10);

    // display driver start
    display.start();

    // lock lvgl
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

    // unlock lvgl
    display.unlock(); 


    sdcardExample(sdcard);
  
    led.init();   

   
             

    while(true) {
        led.setColor(255, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        led.setColor(0, 255, 0); 
        vTaskDelay(pdMS_TO_TICKS(1000));

        led.setColor(0, 0, 255); 
        vTaskDelay(pdMS_TO_TICKS(1000));

        led.clear(); 
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}
