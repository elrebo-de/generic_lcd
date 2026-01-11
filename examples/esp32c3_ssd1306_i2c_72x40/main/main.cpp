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

GenericLcd *lcdPointer;

extern "C" {
    static uint8_t CommCb(u8x8_t *a, uint8_t b, uint8_t c, void *d)
    {
      return lcdPointer->CommCb(a, b, c, d);
    }
    static uint8_t GpioCb(u8x8_t *a, uint8_t b, uint8_t c, void *d)
    {
      return lcdPointer->GpioCb(a, b, c, d);
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(tag, "ESP32C3 SSD1306 I2C 72x40 Example Program");

    /* First configure the I2C Master Bus */
    ESP_LOGI(tag, "I2cMaster");
    I2cMaster i2c(
		std::string("I2C Master Bus"), // tag
		(i2c_port_num_t) -1, // I2C_MASTER_NUM, // i2cPort
		(gpio_num_t) 6, // sclPin
		(gpio_num_t) 5 // sdaPin
    );

    // then add the LCD device to the I2Cmaster bus
    ESP_LOGI(tag, "I2cDevice");
    I2cDevice lcdDevice(
        std::string("LCD Device"), // tag
        std::string("LCD"), // deviceName
        (i2c_addr_bit_len_t) I2C_ADDR_BIT_LEN_7, // devAddrLength
        (uint16_t) 0x3c, // deviceAddress (without R/W bit)
        (uint32_t) 400000 // sclSpeedHz
    );

    ESP_LOGI(tag, "AddDevice");
    i2c.AddDevice(&lcdDevice);

    /* then configure the LCD */
    ESP_LOGI(tag, "GenericLcd");
    GenericLcd myLcd(
		std::string("ESP32C3 SSD1306 I2C 72x40 LCD"), // tag
		&i2c, // I2cMaster instance
		&lcdDevice // i2c device for LCD
    );

    lcdPointer = &myLcd;

    // the setup the u8g2 I2C driver with the callback functions
    ESP_LOGI(tag, "u8g2_Setup_ssd1306_i2c_72x40_er_f");
    //u8g2_Setup_sh1106_i2c_72x40_wise_f(
    u8g2_Setup_ssd1306_i2c_72x40_er_f(
        myLcd.GetU8g2Address(),
        U8G2_R0,
        CommCb,
        GpioCb);  // init u8g2 structure

    // finalize the initialization
    ESP_LOGI(tag, "Begin");
    myLcd.Begin();

    /*
    // dump u8g2 structure and u8x8 structure
    u8g2_t *u8g2Pointer = myLcd.GetU8g2Address();
    //u8x8_t *u8x8Pointer = &(u8g2Pointer->u8x8);
    //u8x8_display_info_t *display_info = u8x8Pointer->display_info;
    printf("u8g2.width: %u\n", u8g2Pointer->width);
    printf("u8g2.height: %u\n", u8g2Pointer->height);
    printf("u8g2.pixel_buf_width: %u\n", u8g2Pointer->pixel_buf_width);
    printf("u8g2.pixel_buf_height: %u\n", u8g2Pointer->pixel_buf_height);
    */

    int i = 0;

    char buffer1[11];
    char buffer2[11];

    ESP_LOGI(tag, "myLcd.IsInitialized: %s", myLcd.IsInitialized() ? "true" : "false");

    while(true) {
        i++;
        if(myLcd.IsInitialized()) {
            myLcd.ClearBuffer();
            myLcd.DrawFrame(0, 0, 72, 40);
            myLcd.SetFont(u8g2_font_resoledmedium_tr);
            myLcd.DrawStr(2, 9, "running for");
            sprintf(buffer1, "%3i", i / 60);
            //printf("%s\n", buffer1);
            myLcd.DrawStr(2, 18, (std::string(buffer1) + std::string(" min.")).c_str());
            sprintf(buffer2, "%3i", i % 60);
            //printf("%s\n", buffer2);
            myLcd.DrawStr(2, 27, (std::string(buffer2) + std::string(" sec.!")).c_str());
            myLcd.DrawFrame(6, 30, 60, 6);
            myLcd.DrawBox(6, 30, i % 60, 6);
            myLcd.SendBuffer();
            // vTaskDelay(pdMS_TO_TICKS(2000));

            ESP_LOGI(tag, "%3i min. and %2i sec.", i / 60, i % 60);
        }

        vTaskDelay(pdMS_TO_TICKS(1000 - 450)); // delay 1 second
    }
}
