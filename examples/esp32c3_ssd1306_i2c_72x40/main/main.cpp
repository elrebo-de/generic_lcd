/*
 * Example program to use an I2C LCD with elrebo-de/generic_lcd
 */

#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "generic_lcd.hpp"
#include "i2c_master.hpp"

#include "esp_log.h"

static const char *tag = "SSD1306 72x40 LCD";

extern "C" void app_main(void)
{
    ESP_LOGI(tag, "ESP32C3 SSD1306 I2C 72x40 Example Program");

    /* First configure the I2C Master Bus */
    I2cMaster i2c(
		std::string("I2C Master Bus"), // tag
		(i2c_port_num_t) 0, // I2C_MASTER_NUM, // i2cPort
		(gpio_num_t) 6, // sclPin
		(gpio_num_t) 5 // sdaPin
    );

    // then add the LCD device to the I2Cmaster bus
    I2cDevice lcdDevice(
        std::string("LCD Device"), // tag
        std::string("LCD"), // deviceName
        (i2c_addr_bit_len_t) I2C_ADDR_BIT_LEN_7, // devAddrLength
        (uint16_t) 0x3c, // deviceAddress (without R/W bit)
        (uint32_t) 50000 // sclSpeedHz
        );

    i2c.AddDevice(&lcdDevice);

    /* then configure the LCD */
    GenericLcd myLcd(
		std::string("ESP32C3 SSD1306 I2C 72x40 LCD"), // tag
		&i2c, // I2cMaster instance
		&lcdDevice // i2c device for LCD
    );

    // the setup the u8g2 I2C driver with the callback functions
    u8g2_Setup_ssd1306_i2c_72x40_er_f(
        myLcd.GetU8g2Address(),
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure

	//u8x8_SetI2CAddress(&(myLcd.GetU8g2Address()->u8x8), lcdDevice.GetConfig().device_address << 1);

    // finalize the initialization
    myLcd.SetupDone();

    int i = 0;

    char buffer[11];

    ESP_LOGI(tag, "myLcd.IsInitialized: %s", myLcd.IsInitialized() ? "true" : "false");

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
