//
// vim: ts=4 et
// Copyright (c) 2025 Petr Vanek, petr@fotoventus.cz
//
/// @file   display_driver.h
/// @author Petr Vanek

/*
   Display driver for ESP32-C6 1.47inch Display Development Board, 172×320, 262K Color, 
   160MHz Running Frequency Single-core Processor, Supports WiFi 6 & Bluetooth

   https://www.waveshare.com/esp32-c6-lcd-1.47.htm
*/


#pragma once

#include "lvgl.h"
#include "esp_lcd_panel_io.h"
#include "driver/ledc.h"  
#include "spi_manager.h" 

class DisplayDriver
{

public:
    enum class DisplayRotation
    {
        Portrait_0,
        Landscape_90,
        Portrait_180,
        Landscape_270
    };

private:
    esp_lcd_panel_handle_t _panelHandle; // LCD panel handle
    esp_lcd_panel_io_handle_t _ioHandle; // LCD IO handle
    ledc_channel_config_t _ledc;
    SemaphoreHandle_t _lvglLock{nullptr};  ///< Semaphore to synchronize access to LVGL.
    TaskHandle_t _lvglTaskHandle{nullptr}; ///< Handle for the LVGL working task.
    DisplayRotation _rotation;
    lv_disp_drv_t _dispDrv;
    lv_disp_draw_buf_t _drawBuffer;
    lv_color_t *_buf1 = nullptr;
    lv_color_t *_buf2 = nullptr;

    static constexpr const char *TAG = "DD";

public:
    DisplayDriver(DisplayRotation r = DisplayRotation::Portrait_0);
    ~DisplayDriver();

    /**
     * @brief Initializes the display bus and IO configurations.
     * @param needInit - true - initialize SPI
     */
    void initBus(bool needInit = true);

    /**
     * @brief Starts the LVGL driver and attaches the display task.
     *
     * @param usStackDepth Stack size for the display task. Default is 4094 bytes.
     * @param uxPriority Priority of the display task. Default is 2.
     * @param coreId Atach to core Id
     */
    void start(const uint32_t usStackDepth = 4094, UBaseType_t uxPriority = 2);

    /**
     * @brief Stop detach LVGL driver and stop display task.
     *
     */
    void stop();

    /**
     * @brief Locks the LVGL UI to prevent simultaneous access.
     *
     * @param timeout_ms Timeout for the lock in milliseconds. Default is -1 (wait indefinitely).
     * @return true if the lock was acquired successfully, false otherwise.
     */
    bool lock(int timeout_ms = -1);

    /**
     * @brief Unlocks the LVGL UI to allow access by other tasks.
     */
    void unlock();

    /**
     *    @brief Backlight.
     *    @param backlight level
     */
    void backLight(uint8_t level);

    void setRotation(DisplayRotation rotation);

private:
    void initSPI();  // Initialize SPI bus
    void initLCD();  // Initialize ST7789 LCD
    void initLVGL(); // Initialize LVGL
    void initBacklight();
    void allocateBuffer();

    static void lvglFlush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
    static bool lgvlFlushReady(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
    static void lgvlPortpdate(lv_disp_drv_t *drv);
    static void lvglWorkingTask(void *arg);
   
};
