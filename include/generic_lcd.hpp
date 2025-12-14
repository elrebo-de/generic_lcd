/*
 * generic_lcd.hpp
 *
 *  Created on: 07.12.2025
 *      Author: christophoberle
 *
 * this work is licenced under the Apache 2.0 licence
 */

#ifndef GENERIC_LCD_HPP_
#define GENERIC_LCD_HPP_

#include <string>
#include "i2c_master.hpp"

extern "C" {
    #include "u8g2_esp32_hal.h"
}

/* class GenericLcd
   driver for an LCD display using u8g2 library and u8g2-hal-esp-idf
   currently only for I2C drivers
   for the SPI drivers an other constructor must be defined

   The I2c master bus and the device registration for the LCD device are done with I2cMaster class
   before the GenericLcd instance is created.
   The pointer to the I2cMaster instance and the deviceName of the LCD is needed in the constructor
   of GenericLcd.
*/

class GenericLcd {
public:
    // Constructor of I2C LCD
	GenericLcd(std::string tag,
	           I2cMaster *i2c, // i2c master instance
	           std::string deviceName); // deviceName of LCD
	virtual ~GenericLcd();
	void SetupDone();
	u8g2_t *GetU8g2Address();
	bool IsInitialized();

	/* u8g2 function wrappers
	   with:
	               GenericLcd myLcd(...);
	   instead of this call:
	               ESP_LOGD(tag, "u8g2_SetPowerSave");
                   u8g2_SetPowerSave(myLcd.GetU8g2Address(), 0);  // wake up display
       this wrapper method of GenericLcd can be used:
                   myLcd.SetPowerSave(0); // wake up display
    */

    // from u8x8_display.c
    void SetPowerSave(uint8_t is_enable);
    void SetFlipMode(uint8_t mode);
    void SetContrast(uint8_t value);
    //void ClearDisplayWithTile(const uint8_t *buf)  U8X8_NOINLINE;
    //void ClearDisplay();	// this does not work for u8g2 in some cases
    //void FillDisplay();
    //void RefreshDisplay();	// make RAM content visible on the display (Dec 16: SSD1606 only)
    //void ClearLine(uint8_t line);

    // from u8g2_buffer.c
    void SendBuffer();
    void ClearBuffer();
    void FirstPage();
    uint8_t NextPage();

    // from u8g2_font.c
    void SetFont(const uint8_t  *font);
    void SetFontMode(uint8_t is_transparent);
    void SetFontDirection(uint8_t dir);
    u8g2_uint_t DrawStr(u8g2_uint_t x, u8g2_uint_t y, const char *str);
    //u8g2_uint_t DrawStrX2(u8g2_uint_t x, u8g2_uint_t y, const char *str);
    u8g2_uint_t DrawUTF8(u8g2_uint_t x, u8g2_uint_t y, const char *str);
    //u8g2_uint_t DrawUTF8X2(u8g2_uint_t x, u8g2_uint_t y, const char *str);
    //u8g2_uint_t DrawExtendedUTF8(u8g2_uint_t x, u8g2_uint_t y, uint8_t to_left, u8g2_kerning_t *kerning, const char *str);
    //u8g2_uint_t DrawExtUTF8(u8g2_uint_t x, u8g2_uint_t y, uint8_t to_left, const uint16_t *kerning_table, const char *str);

    // from u8g2_box.c
    void DrawBox(u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h);
    void DrawFrame(u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h);
    void DrawRBox(u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h, u8g2_uint_t r);
    void DrawRFrame(u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h, u8g2_uint_t r);

private:
    std::string tag = "GenericLcd";
    I2cMaster *i2c;
    std::string deviceName;

    u8g2_t u8g2;  // a structure which will contain all the data for one display
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT; // hardware abstraction layer for u8g2 on ESP32 processors

    bool initialized;
};

#endif /* GENERIC_LCD_HPP_ */
