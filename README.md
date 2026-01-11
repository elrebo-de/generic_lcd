# Generic LCD component

This repository contains an ESP-IDF component for a generic LCD. It runs on
any ESP32 processor and is built using the ESP-IDF build system in version 5.5+.

The component is implemented as C++ class `GenericLcd`.

It uses the u8g2 library (from https://github.com/olikraus/u8g2.git) 
and the u8g2-hal-esp-idf component (from https://github.com/mkfrey/u8g2-hal-esp-idf.git) pull request 22 (version: refs/pull/22/head)

It is using the new I2C driver (set CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2=y in sdkconfig)

Currently only those drivers from u8g2 using the I2C bus can be used! Drivers using the SPI bus are not yet implemented.

## Connecting the component

The constructor of class `GenericLcd` has four parameters:

| Parameter  | Type of Parameter | Usage                                        |
|:-----------|:------------------|:---------------------------------------------|
| tag        | std::string       | the tag to be used in the ESP log            |
| sclPin     | gpio_num_t        | the gpio number of the I2C scl pin           |
| sdaPin     | gpio_num_t        | the gpio number of the I2C sda pin           |
| i2cAddress | uint8_t           | the address of the LCD on the I2C bus        |

# Usage

## API
The API of the component is located in the include directory ```include/generic_lcd.hpp``` and defines the
C++ class ```GenericLcd```

Currently only those drivers from u8g2 using the I2C bus can be used! Drivers using the SPI bus are not yet implemented.

```C++
/* class GenericLcd
   driver for an LCD display using u8g2 library and u8g2-hal-esp-idf
   currently only for I2C drivers
   for the SPI drivers an other constructor must be defined
*/
class GenericLcd {
public:
    // Constructor of I2C LCD
	GenericLcd(std::string tag,
	           gpio_num_t sclPin, // i2c scl Pin
	           gpio_num_t sdaPin, // i2c sda Pin
	           uint8_t i2cAddress); // i2c address of lcd device
	virtual ~GenericLcd();
	void Begin();
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
    gpio_num_t sclPin;
    gpio_num_t sdaPin;
    uint8_t i2cAddress; // address of the LCD device on theI2C bus

    u8g2_t u8g2;  // a structure which will contain all the data for one display
    u8g2_esp32_hal_t u8g2_esp32_hal; // hardware abstraction layer for u8g2 on ESP32 processors

    bool initialized;
};
```
Initialization of the GenericLcd class is a three-step process:
* create an instance of GenericLcd class
* call u8g2_setup function for the LCD panel
* call Method GenericLcd::SetupDone()

```
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
```

# License

This component is provided under the Apache 2.0 license.
