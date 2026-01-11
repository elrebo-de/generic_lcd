/*
 * generic_lcd.hpp
 *
 *      Author: christophoberle
 *
 * this work is licenced under the Apache 2.0 licence
 */

#ifndef GENERIC_LCD_HPP_
#define GENERIC_LCD_HPP_

#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_master.hpp"

#include "esp_log.h"

#include "u8g2.h"
#include "u8g2_hal_espidf.hpp"

/* class U8g2_Hal:I2C
   Subclass of u8g2_hal::U8g2_Hal_I2C
   this subclass uses an already connected i2c device
*/

class XU8g2_Hal_I2C : public u8g2_hal::U8g2_Hal_I2C {
    /// @brief The second step of initializing the HAL
    /// @note needs to be done at runtime. The first step is the constructor.
    /// @param deviceHandle This must be already initialized and added to the bus!
    public:
        XU8g2_Hal_I2C() : u8g2_hal::U8g2_Hal_I2C(1000, 32, {})
        {
        }

        esp_err_t Begin(i2c_master_dev_handle_t devHandle)
        {
          m_hDisp = devHandle;
          return ESP_OK;
        }
};

/* class GenericLcd
   driver for an LCD display using u8g2 library and u8g2-hal-esp-idf
   currently only for I2C drivers
   for the SPI drivers an other constructor must be defined

   The I2c master device handle is registered with I2cMaster class
   before the GenericLcd instance is created.
   The pointer to the I2cMaster and I2CDevice instances are needed in the constructor
   of GenericLcd.
*/
class GenericLcd {
public:
    // Constructor of I2C LCD V2
	GenericLcd(std::string tag,
	           I2cMaster *i2cMaster, // i2c master instance
	           I2cDevice *i2cDevice); // i2c device for LCD
	virtual ~GenericLcd();
	void Begin();
	u8g2_t *GetU8g2Address();
	bool IsInitialized();

    /// @note I attempted to use C++ lambda and std::bind but
    ///   failed to convert it into plain C function pointer,
    ///   so this boilerplate is the last resort.
    uint8_t CommCb(u8x8_t *a, uint8_t b, uint8_t c, void *d)
    {
      //vTaskDelay(pdMS_TO_TICKS(10));
      return u8g2hal.i2c_byte_cb(a, b, c, d);
    }
    uint8_t GpioCb(u8x8_t *a, uint8_t b, uint8_t c, void *d)
    {
      //vTaskDelay(pdMS_TO_TICKS(10));
      return u8g2hal.gpio_and_delay_cb(a, b, c, d);
    }

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
    I2cMaster *i2cMaster;
    I2cDevice *i2cDevice;

    u8g2_t u8g2;  // a structure which will contain all the data for one display
    XU8g2_Hal_I2C u8g2hal;

    bool initialized;
};

#endif /* GENERIC_LCD_HPP_ */
