//
// vim: ts=4 et
// Copyright (c) 2025 Petr Vanek, petr@fotoventus.cz
//
/// @file   rgb_led.h
/// @author Petr Vanek


#pragma once

#include <stdint.h>
#include <esp_log.h>
#include "led_strip.h"

/**
 * @brief Class for controlling an RGB LED strip.
 */
class RGBLed {
private:
    static constexpr const char *TAG = "RGBLed"; ///< Logging tag.
    int gpio_;  
    led_strip_handle_t led_strip_;               ///< Handle for the LED strip.
                                    ///< GPIO pin used for LED strip control.

public:
    /**
     * @brief Constructor for the RGB LED strip.
     * @param gpio The GPIO pin to which the LED strip is connected.
     */
    explicit RGBLed(int gpio) : gpio_(gpio), led_strip_(nullptr) {}

    /**
     * @brief Initializes the LED strip.
     */
    void init() {
        ESP_LOGI(TAG, "Initializing LED strip on GPIO %d", gpio_);

        // LED strip configuration
        led_strip_config_t strip_config = {
            .strip_gpio_num = gpio_,
            .max_leds = 1, 
        };

        led_strip_rmt_config_t rmt_config;
        rmt_config.resolution_hz = 10 * 1000 * 1000;  // 10 MHz
        rmt_config.flags.with_dma = false;
        rmt_config.clk_src = RMT_CLK_SRC_DEFAULT; // Ensure clock source is correctly set
        rmt_config.mem_block_symbols = 64;  // Default memory allocation for the RMT driver
        
         
        // Create LED strip device
        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_));

        // Turn off all LEDs (clear strip)
        clear();
    }

    /**
     * @brief Sets the color of the LED.
     * @param red Red component (0-255).
     * @param green Green component (0-255).
     * @param blue Blue component (0-255).
     */
    void setColor(uint8_t red, uint8_t green, uint8_t blue) {
        led_strip_set_pixel(led_strip_, 0, red, green, blue);
        led_strip_refresh(led_strip_);
    }

    /**
     * @brief Turns off the LED (sets the color to black).
     */
    void clear() {
        led_strip_clear(led_strip_);
    }
};
