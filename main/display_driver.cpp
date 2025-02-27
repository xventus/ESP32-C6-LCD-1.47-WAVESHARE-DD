//
// vim: ts=4 et
// Copyright (c) 2025 Petr Vanek, petr@fotoventus.cz
//
/// @file   display_driver.h
/// @author Petr Vanek


#include "display_driver.h"
#include "hardware.h"
#include "esp_lcd_panel_commands.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
    
#include "driver/gpio.h"      
#include "driver/spi_master.h" 
#include "esp_lcd_panel_io.h"  
#include "esp_lcd_panel_ops.h" 
#include "esp_lcd_panel_vendor.h"


DisplayDriver::DisplayDriver(DisplayRotation r) : _rotation(r)
{
    _panelHandle = nullptr;
    _ioHandle = nullptr;
    memset(&_ledc, 0, sizeof(_ledc));
    allocateBuffer();
    setRotation(_rotation);
}


void DisplayDriver::allocateBuffer()
{
    if (_buf1) free(_buf1);
    if (_buf2) free(_buf2);

    uint16_t buf_width = (_rotation == DisplayRotation::Landscape_90 || _rotation == DisplayRotation::Landscape_270) ? HW_LCD_V : HW_LCD_H;

    _buf1 = (lv_color_t *)malloc(buf_width * 10 * sizeof(lv_color_t));
    _buf2 = (lv_color_t *)malloc(buf_width * 10 * sizeof(lv_color_t));

    if (!_buf1 || !_buf2) {
        ESP_LOGE(TAG, "Failed to allocate memory for LVGL buffers!");
        _buf1 = _buf2 = nullptr;
        return;
    }

    lv_disp_draw_buf_init(&_drawBuffer, _buf1, _buf2, buf_width * 10);

    ESP_LOGI(TAG, "Reallocated buffers for rotation: width=%d, size=%d bytes per buffer", buf_width, buf_width * 10 * sizeof(lv_color_t));
}



DisplayDriver::~DisplayDriver()
{
   stop();
 
   if (_buf1) free(_buf1);
   if (_buf2) free(_buf2);
}

bool DisplayDriver::lock(int timeout_ms)
{
 //   ESP_LOGI(TAG, "LVGL lock");
    const TickType_t tc = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    bool result = xSemaphoreTakeRecursive(_lvglLock, tc) == pdTRUE;
    
    if (!result) {
        ESP_LOGW(TAG, "LVGL lock timeout, retrying...");
        vTaskDelay(pdMS_TO_TICKS(5));  // Short delay before retry
        return false;
    }
    
    return result;
}



void DisplayDriver::unlock()
{
    //ESP_LOGI(TAG, "LVGL unlocked.");
    xSemaphoreGiveRecursive(_lvglLock);
}


void DisplayDriver::stop()
{
    if (_lvglTaskHandle != nullptr)
    {
        vTaskDelete(_lvglTaskHandle);
        _lvglTaskHandle = nullptr;
    }

    if (_lvglLock != nullptr)
    {
        vSemaphoreDelete(_lvglLock);
        _lvglLock = nullptr;
    }

    ESP_LOGI(TAG, "LVGL task stopped.");
}


void DisplayDriver::start(const uint32_t usStackDepth, UBaseType_t uxPriority)
{
    if (_lvglTaskHandle != nullptr) {
        ESP_LOGW(TAG, "LVGL task is already running!");
        return;
    }

    _lvglLock = xSemaphoreCreateRecursiveMutex();
    if (_lvglLock == nullptr) {
        ESP_LOGE(TAG, "Failed to create LVGL lock mutex.");
        return;
    }

    auto result = xTaskCreate(lvglWorkingTask, "LVGLTSK", usStackDepth, this, uxPriority, &_lvglTaskHandle);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LVGL task.");
        _lvglTaskHandle = nullptr;
    }

    ESP_LOGI(TAG, "LVGL thread started");
}

void DisplayDriver::lvglWorkingTask(void *arg)
{
    DisplayDriver *dd = static_cast<DisplayDriver *>(arg);

    while (true)
    {
      
        if (dd->_lvglLock == nullptr) {
            ESP_LOGE(TAG, "LVGL lock is NULL!");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (!dd->lock(10)) { 
            ESP_LOGW(TAG, "LVGL lock timeout, skipping this cycle.");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        lv_tick_inc(10);  
        lv_timer_handler();

        dd->unlock();
  
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void DisplayDriver::lvglFlush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    DisplayDriver *instance = (DisplayDriver *)drv->user_data;
    lv_color_t first_pixel = color_map[0];
    /*
    ESP_LOGI(TAG, "First pixel: R=%d, G=%d, B=%d",
             first_pixel.ch.red,
             first_pixel.ch.green,
             first_pixel.ch.blue);
    */
    {
        SPILockGuard lock;
        esp_err_t ret = esp_lcd_panel_draw_bitmap(instance->_panelHandle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error drawing bitmap: %d", ret);
        }
    }

    lv_disp_flush_ready(drv);
}


void DisplayDriver::initLVGL()
{
    ESP_LOGI(TAG, "Initializing LVGL");
    lv_init();
    lv_disp_draw_buf_init(&_drawBuffer, _buf1, _buf2, HW_LCD_H * 10);
    lv_disp_drv_init(&_dispDrv);
    if (_rotation == DisplayRotation::Portrait_0 || _rotation == DisplayRotation::Portrait_180)
    {  
        _dispDrv.hor_res = HW_LCD_H;
        _dispDrv.ver_res = HW_LCD_V;
    } else {
        _dispDrv.hor_res = HW_LCD_V;
        _dispDrv.ver_res = HW_LCD_H;
    }

    _dispDrv.flush_cb = lvglFlush;
    _dispDrv.drv_update_cb = lgvlPortpdate;
    _dispDrv.draw_buf = &_drawBuffer;
    _dispDrv.user_data = this;
    lv_disp_drv_register(&_dispDrv);
}


 void DisplayDriver::setRotation(DisplayRotation rotation)
{
    if (_lvglTaskHandle) {
        vTaskSuspend(_lvglTaskHandle); // Pause LVGL task during rotation
    }

    _rotation = rotation;  // Store new rotation value

    uint8_t madctl_value = 0x00;
    switch (rotation)
    {
    case DisplayRotation::Portrait_0:
        madctl_value = 0x40;
        break;
    case DisplayRotation::Landscape_90:
        madctl_value = 0x20;
        break;
    case DisplayRotation::Portrait_180:
        madctl_value = 0x80;
        break;
    case DisplayRotation::Landscape_270:
        madctl_value = 0xE0;
        break;
    }

    esp_lcd_panel_io_tx_param(_ioHandle, 0x36, &madctl_value, 1);

    allocateBuffer(); 

    if (_lvglTaskHandle) {
        vTaskResume(_lvglTaskHandle); 
    }

    ESP_LOGI(TAG, "Display rotated to mode: %d", (int)rotation);
}

void DisplayDriver::lgvlPortpdate(lv_disp_drv_t *drv)
{
    ESP_LOGI(TAG, "lgvlPortpdate called, drv->rotated = %d", (int)drv->rotated);

    DisplayDriver *instance = (DisplayDriver *)drv->user_data;
    DisplayRotation rotation = DisplayRotation::Portrait_0;
    static DisplayRotation lastRotation = DisplayRotation::Portrait_0;

    switch (drv->rotated)
    {
    case LV_DISP_ROT_NONE:
        rotation = DisplayRotation::Portrait_0;
        break;
    case LV_DISP_ROT_90:
        rotation = DisplayRotation::Landscape_90;
        break;
    case LV_DISP_ROT_180:
        rotation = DisplayRotation::Portrait_180;
        break;
    case LV_DISP_ROT_270:
        rotation = DisplayRotation::Landscape_270;
        break;
    default:
        ESP_LOGW(TAG, "Unknown rotation mode.");
        return;
    }

    if (rotation == lastRotation) {
        ESP_LOGI(TAG, "Rotation unchanged, skipping update.");
        return;
    }

    lastRotation = rotation;

    ESP_LOGI(TAG, "Setting rotation to: %d", (int)rotation);
    instance->setRotation(rotation);

    // Update LVGL resolution
    lv_disp_t *disp = lv_disp_get_default();
    if (!disp) {
        ESP_LOGE(TAG, "No display found!");
        return;
    }

    lv_disp_drv_t *disp_drv = disp->driver;
    if (!disp_drv) {
        ESP_LOGE(TAG, "No display driver found!");
        return;
    }

    disp_drv->hor_res = (rotation == DisplayRotation::Portrait_0 || rotation == DisplayRotation::Portrait_180) ? HW_LCD_H : HW_LCD_V;
    disp_drv->ver_res = (rotation == DisplayRotation::Portrait_0 || rotation == DisplayRotation::Portrait_180) ? HW_LCD_V : HW_LCD_H;

    lv_disp_drv_update(disp, disp_drv);

    instance->allocateBuffer();
    ESP_LOGI(TAG, "Display rotation updated to: %d (hor_res=%d, ver_res=%d)", (int)rotation, disp_drv->hor_res, disp_drv->ver_res);
}


void DisplayDriver::initBus(bool needInit)
{

    ESP_LOGI(TAG, "Initializing DisplayDriver");

    if (needInit)
    {
        initSPI();
    }

    initLCD();
    initBacklight();
    initLVGL();
    backLight(75);
}

void DisplayDriver::initSPI()
{
    ESP_LOGI(TAG, "Initializing SPI bus...");
    spi_bus_config_t buscfg = {
        .mosi_io_num = HW_LCD_MOSI,
        .miso_io_num = HW_LCD_MISO,
        .sclk_io_num = HW_LCD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000};

    esp_err_t ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI bus initialization failed: %d", ret);
    }
    ESP_LOGI(TAG, "SPI bus initialized successfully!");
}


bool DisplayDriver::lgvlFlushReady(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}


void DisplayDriver::initLCD()
{
    ESP_LOGI(TAG, "Initializing LCD...");

    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = HW_LCD_CS,
        .dc_gpio_num = HW_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 10 * 1000 * 1000,
        .trans_queue_depth = 10,
        .on_color_trans_done = lgvlFlushReady,
        .user_ctx = &_dispDrv,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8};

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &_ioHandle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = HW_LCD_RST,
        .color_space = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16};

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(_ioHandle, &panel_config, &_panelHandle));

    //  Add offsets to correct misalignment
    if (_rotation == DisplayRotation::Portrait_0 || _rotation == DisplayRotation::Portrait_180)  ESP_ERROR_CHECK(esp_lcd_panel_set_gap(_panelHandle, 34, 0)); // Adjust for Waveshare module
        else ESP_ERROR_CHECK(esp_lcd_panel_set_gap(_panelHandle, 0, 34));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(_panelHandle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(_panelHandle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(_panelHandle, true, false)); // portrait OK
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(_panelHandle, true));
    esp_lcd_panel_io_tx_param(_ioHandle, LCD_CMD_SLPOUT, NULL, 0); //??
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 0x36
    +------+-----+-------------------------------+
    | Bit  | Name| Effect                        |
    +------+-----+-------------------------------+
    |  7   | MH  | Horizontal refresh order      |
    |  6   | RGB | RGB/BGR order swap            |
    |  5   | ML  | Vertical refresh order        |
    |  4   | MV  | Row/Column exchange (90° rot) |
    |  3   | MX  | Mirror X-axis (flip horiz.)   |
    |  2   | MY  | Mirror Y-axis (flip vert.)    |
    | 1-0  |  -  | Reserved (always 0)           |
    +------+-----+-------------------------------+

    +------------+---------------------------+--------+
    |  MADCTL    | Effect                    | Value  |
    +------------+---------------------------+--------+
    | Normal     | Default (no flip, no rot) |  0x00  |
    | Mirror X   | Flips horizontally        |  0x40  |
    | Mirror Y   | Flips vertically          |  0x80  |
    | 180° Flip  | Both X & Y mirror         |  0xC0  |
    | Rotate 90° | Row/Column exchange       |  0x20  |
    | Rotate 270°| Row/Col + Mirror X & Y    |  0xA0  |
    +------------+---------------------------+--------+

    */
   setRotation(_rotation);
   /*
    if (_rotation == DisplayRotation::Portrait_0) esp_lcd_panel_io_tx_param(io_handle, 0x36, (uint8_t[]){0x40}, 1); // USB bottom - portrait
    else if (_rotation == DisplayRotation::Portrait_180) esp_lcd_panel_io_tx_param(io_handle, 0x36, (uint8_t[]){0x80}, 1);  // USB top - portrait 
    else if (_rotation == DisplayRotation::Landscape_90) esp_lcd_panel_io_tx_param(io_handle, 0x36, (uint8_t[]){0x20}, 1);  // USB right - landscape
    else if (_rotation == DisplayRotation::Portrait_0) esp_lcd_panel_io_tx_param(io_handle, 0x36, (uint8_t[]){0xE0}, 1);  // USB left - landscape
   */
   
    /* 0x3A
    +--------+-----------------+------------------------------+
    | Value  | Color Mode      | Bits per Pixel (BPP)         |
    +--------+-----------------+------------------------------+
    | 0x55   | RGB565         | 16-bit (5R + 6G + 5B)         |
    | 0x66   | RGB666         | 18-bit (6R + 6G + 6B)         |
    | 0x77   | RGB888 (if supported) | 24-bit (8R + 8G + 8B)  |
    +--------+-----------------+------------------------------+
    */
    esp_lcd_panel_io_tx_param(_ioHandle, 0x3A, (uint8_t[]){0x55}, 1);

    // RAMCTRL (B0h): RAM Control
    esp_lcd_panel_io_tx_param(_ioHandle, 0xB0, (uint8_t[]){0x00, 0xE8}, 2);

    // PORCTRL (B2h): Porch Setting
    esp_lcd_panel_io_tx_param(_ioHandle, 0xB2, (uint8_t[]){0x0c, 0x0c, 0x00, 0x33, 0x33}, 5);

    // GCTRL (B7h): Gate Control     VGH (V) 13.65,  VGL (V) 10.43V
    esp_lcd_panel_io_tx_param(_ioHandle, 0xB7, (uint8_t[]){0x75}, 1);
    /* VCOM Setting, VCOM=1.175V */

    // VCOMS (BBh): VCOMS Setting
    esp_lcd_panel_io_tx_param(_ioHandle, 0xBB, (uint8_t[]){0x1A}, 1);

    // LCMCTRL (C0h): LCM Control
    esp_lcd_panel_io_tx_param(_ioHandle, 0xC0, (uint8_t[]){0x80}, 1);

    // VDVVRHEN (C2h): VDV and VRH Command Enable
    esp_lcd_panel_io_tx_param(_ioHandle, 0xC2, (uint8_t[]){0x01, 0xff}, 2);

    // VRHS (C3h): VRH Set
    esp_lcd_panel_io_tx_param(_ioHandle, 0xC3, (uint8_t[]){0x13}, 1);

    /// VDVS (C4h): VDV Set
    esp_lcd_panel_io_tx_param(_ioHandle, 0xC4, (uint8_t[]){0x20}, 1);

    // FRCTRL2 (C6h): Frame Rate Control in Normal Mode
    esp_lcd_panel_io_tx_param(_ioHandle, 0xC6, (uint8_t[]){0x0F}, 1);

    // PWCTRL1 (D0h): Power Control 1
    esp_lcd_panel_io_tx_param(_ioHandle, 0xD0, (uint8_t[]){0xA4, 0xA1}, 1);

    // PVGAMCTRL (E0h): Positive Voltage Gamma Control
    esp_lcd_panel_io_tx_param(_ioHandle, 0xE0, (uint8_t[]){0xD0, 0x0D, 0x14, 0x0D, 0x0D, 0x09, 0x38, 0x44, 0x4E, 0x3A, 0x17, 0x18, 0x2F, 0x30}, 14);

    // NVGAMCTRL (E1h): Negative Voltage Gamma Control
    esp_lcd_panel_io_tx_param(_ioHandle, 0xE1, (uint8_t[]){0xD0, 0x09, 0x0F, 0x08, 0x07, 0x14, 0x37, 0x44, 0x4D, 0x38, 0x15, 0x16, 0x2C, 0x2E}, 14);

    // INVON (21h): Display Inversion On
    esp_lcd_panel_io_tx_param(_ioHandle, 0x21, NULL, 0);

    // DISPON (29h): Display On
    esp_lcd_panel_io_tx_param(_ioHandle, 0x29, NULL, 0);

    // RAMWR (2Ch): Memory Write
    esp_lcd_panel_io_tx_param(_ioHandle, 0x2C, NULL, 0);

}

void DisplayDriver::backLight(uint8_t level)
{
    if (level > 100)
        level = 100;
    uint16_t Duty = LEDC_MAX_DUTY - (81 * (100 - level));
    if (level == 0)
        Duty = 0;

    ledc_set_duty(_ledc.speed_mode, _ledc.channel, Duty);
    ledc_update_duty(_ledc.speed_mode, _ledc.channel);
}

void DisplayDriver::initBacklight()
{
    ESP_LOGI(TAG, "Initializing LCD backlight...");

    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = 1ULL << HW_LCD_BL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    // Configure LEDC Timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    _ledc.channel = LEDC_CHANNEL_0;
    _ledc.duty = 0;
    _ledc.gpio_num = HW_LCD_BL;
    _ledc.speed_mode = LEDC_LOW_SPEED_MODE;
    _ledc.timer_sel = LEDC_TIMER_0;

    ESP_ERROR_CHECK(ledc_channel_config(&_ledc));
    ledc_fade_func_install(0);
    ESP_LOGI(TAG, "Backlight initialized.");
}
