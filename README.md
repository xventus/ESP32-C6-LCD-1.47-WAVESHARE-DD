# ESP32-C6 1.47inch Display Development Board 

172×320, 262K Color

Display driver for waveshare 1.47inch display and LGVL for ESP IDF framework. 

Additional work with SD card and RGB led. In case there is no need to use the SD card it is necessary to call display.initBus(true). If the SD card is to be used it is first necessary to initialize the SPI for sdcard.initBus(true) and then it is not necessary to initialize display.initBus(false).   

If the display is shifted, it is necessary to correct it by changing the settings in the esp_lcd_panel_set_gap(_panelHandle, XXX, YYY) register. 

Because the SPI bus is shared between the LCD and the SD, there is an SPI manager that controls access to the SPI. Although this module is RISC V and has one core, there is synchronization for transfer to the dual core ESP.

# Module Information

 https://www.waveshare.com/esp32-c6-lcd-1.47.htm

#Hardware

Pin Connection


| LCD PIN    | ESP32-C6   |
|------------|------------|
| MOSI - DIN | GPIO 6     |
| SCLK       | GPIO 7     |
| LCD_CS     | GPIO 14    |
| LCD_DC     | GPIO 15    |
| LCD_RST    | GPIO 21    |
| LCD_BL     | GPIO 22    |



| SD CARD PIN| ESP32-C6   |
|------------|------------|
| SD_MOSI    | GPIO 6     |
| SD_MISO    | GPIO 5     |
| SD_CLK     | GPIO 7     |
| SD_CS      | GPIO 4     |



| LED        | ESP32-C6   |
|------------|------------|
| LED RGB    |  GPIO 8    |
