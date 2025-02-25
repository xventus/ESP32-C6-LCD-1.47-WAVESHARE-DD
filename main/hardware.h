//
// vim: ts=4 et
// Copyright (c) 2025 Petr Vanek, petr@fotoventus.cz
//
/// @file   hardware.h
/// @author Petr Vanek

#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"


/*


   Board ESP32-C6 1.47inch Display Development Board, 172×320, 262K Color, 
   160MHz Running Frequency Single-core Processor, Supports WiFi 6 & Bluetooth

   https://www.waveshare.com/esp32-c6-lcd-1.47.htm

+------------+------------+
| LCD PIN    | ESP32-C6   |
+------------+------------+
| MOSI - DIN | GPIO 6     |
| SCLK       | GPIO 7     |
| LCD_CS     | GPIO 14    |
| LCD_DC     | GPIO 15    |
| LCD_RST    | GPIO 21    |
| LCD_BL     | GPIO 22    |
+------------+------------+

+------------+------------+
| SD CARD PIN| ESP32-C6   |
+------------+------------+
| SD_MOSI    | GPIO 6     |
| SD_MISO    | GPIO 5     |
| SD_CLK     | GPIO 7     |
| SD_CS      | GPIO 4     |
+------------+------------+

+------------+------------+
| LED        | ESP32-C6   |
+------------+------------+
| LED RGB    |  GPIO 8    |
+------------+------------+



*/

#define HW_LCD_H            172
#define HW_LCD_V            320

#define HW_LCD_MOSI         6 
#define HW_LCD_MISO         5
#define HW_LCD_CLK          7
#define HW_LCD_CS           14 
#define HW_LCD_DC           15 
#define HW_LCD_RST          21 
#define HW_LCD_BL           22 


#define LCD_HOST            SPI2_HOST  // SPI
#define LEDC_RESOLUTION_RATIO   LEDC_TIMER_13_BIT
#define LEDC_MAX_DUTY          ((1 << LEDC_RESOLUTION_RATIO) - 1)

