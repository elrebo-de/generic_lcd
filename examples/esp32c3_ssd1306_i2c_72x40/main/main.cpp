/*
 * Example program to use an I2C LCD with elrebo-de/generic_lcd
 */

#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "generic_lcd.hpp"

#include "esp_log.h"

static const char *tag = "SSD1306 72x40 LCD";

extern "C" void app_main(void)
{
    ESP_LOGI(tag, "ESP32C3 SSD1306 I2C 72x40 Example Program");

    /* Configure the LCD */
    GenericLcd myLcd(
		std::string("ESP32C3 SSD1306 I2C 72x40 LCD"), // tag
		(gpio_num_t) 6, // sclPin
		(gpio_num_t) 5, // sdaPin
		(uint8_t) 0x78 // i2cAddress
    );

    u8g2_Setup_ssd1306_i2c_72x40_er_f(
        myLcd.GetU8g2Address(),
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure

    myLcd.SetupDone();

    int i = 0;

    char buffer[11];

    while(true) {
        i++;
        if(myLcd.IsInitialized()) {
            myLcd.SetPowerSave(0);  // wake up display
            myLcd.ClearBuffer();
            myLcd.DrawFrame(0, 0, 72, 40);

            myLcd.SetFont(u8g2_font_resoledmedium_tr);
            myLcd.DrawStr(2, 9, "running for");

            sprintf(buffer, "%3i", i / 60);
            myLcd.DrawStr(2, 18, (std::string(buffer) + std::string(" min.")).c_str());
            sprintf(buffer, "%3i", i % 60);
            myLcd.DrawStr(2, 27, (std::string(buffer) + std::string(" sec.!")).c_str());

            myLcd.DrawBox(6, 30, i % 60, 6);
            myLcd.DrawFrame(6, 30, 60, 6);

            myLcd.SendBuffer();

            ESP_LOGI(tag, "%3i min. and %2i sec.", i / 60, i % 60);
        }

        vTaskDelay((1000 - 80) / portTICK_PERIOD_MS); // delay 1 second
    }
}
