/*
 * generic_lcd.cpp
 *
 *      Author: christophoberle
 *
 * this work is licenced under the Apache 2.0 licence
 */

#include "u8g2.h"
#include "u8g2_hal_espidf.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_idf_version.h"

#include "generic_lcd.hpp"

GenericLcd::GenericLcd(std::string tag,
                       I2cMaster *i2cMaster, // i2c master instance
                       I2cDevice *i2cDevice // i2c device for LCD
) {
	this->tag = tag;
    this->i2cMaster = i2cMaster;
    this->i2cDevice = i2cDevice;
    this->initialized = false;
}

GenericLcd::~GenericLcd() {
    //if(this-initialized == true) {
    //    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    //    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    //    ESP_LOGD(this->tag, "I2C de-initialized successfully");
    //}
}

void GenericLcd::Begin() {
    // set i2c device handle
    u8g2hal.Begin(this->i2cDevice->GetHandle());
    u8g2_InitDisplay(&(this->u8g2));  // send init sequence to the display, display is in sleep mode after this call
    this->SetPowerSave(0);  // wake up display
    this->initialized = true;
}

u8g2_t *GenericLcd::GetU8g2Address() {
	return &(this->u8g2);
}

bool GenericLcd::IsInitialized() {
	return this->initialized;
}

void GenericLcd::SetPowerSave(uint8_t is_enable) {
	ESP_LOGI(this->tag.c_str(), "SetPowerSave(%u)", is_enable);
	u8g2_SetPowerSave(&(this->u8g2), is_enable);
}

void GenericLcd::SendBuffer() {
	ESP_LOGI(this->tag.c_str(), "SendBuffer");
	u8g2_SendBuffer(&(this->u8g2));
}

void GenericLcd::ClearBuffer() {
	ESP_LOGI(this->tag.c_str(), "ClearBuffer");
	u8g2_ClearBuffer(&(this->u8g2));
}

void GenericLcd::FirstPage() {
	ESP_LOGI(this->tag.c_str(), "FirstPage");
	u8g2_FirstPage(&(this->u8g2));
}

uint8_t GenericLcd::NextPage() {
	ESP_LOGI(this->tag.c_str(), "NextPage");
	return u8g2_NextPage(&(this->u8g2));
}

void GenericLcd::SetFont(const uint8_t  *font) {
	ESP_LOGI(this->tag.c_str(), "SetFont");
	u8g2_SetFont(&(this->u8g2), font);
}

void GenericLcd::SetFontMode(uint8_t  is_transparent) {
	ESP_LOGI(this->tag.c_str(), "SetFontMode(%u)", is_transparent);
	u8g2_SetFontMode(&(this->u8g2), is_transparent);
}

void GenericLcd::SetFontDirection(uint8_t  dir) {
	ESP_LOGI(this->tag.c_str(), "SetFontDirection(%u)", dir);
	u8g2_SetFontMode(&(this->u8g2), dir);
}

u8g2_uint_t GenericLcd::DrawStr(u8g2_uint_t x, u8g2_uint_t y, const char *str){
	ESP_LOGI(this->tag.c_str(), "DrawStr(%u, %u, \"%s\")", x, y, str);
	return u8g2_DrawStr(&(this->u8g2), x, y, str);
}

u8g2_uint_t GenericLcd::DrawUTF8(u8g2_uint_t x, u8g2_uint_t y, const char *str){
	ESP_LOGI(this->tag.c_str(), "DrawUTF8(%u, %u, \"%s\")", x, y, str);
	return u8g2_DrawUTF8(&(this->u8g2), x, y, str);
}

void GenericLcd::DrawBox(u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h){
	ESP_LOGI(this->tag.c_str(), "DrawBox(%u, %u, %u, %u)", x, y, w, h);
	u8g2_DrawBox(&(this->u8g2), x, y, w, h);
}

void GenericLcd::DrawFrame(u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h){
	ESP_LOGI(this->tag.c_str(), "DrawFrame(%u, %u, %u, %u)", x, y, w, h);
	u8g2_DrawFrame(&(this->u8g2), x, y, w, h);
}

void GenericLcd::DrawRBox(u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h, u8g2_uint_t r){
	ESP_LOGI(this->tag.c_str(), "DrawRBox(%u, %u, %u, %u, %u)", x, y, w, h, r);
	u8g2_DrawRBox(&(this->u8g2), x, y, w, h, r);
}

void GenericLcd::DrawRFrame(u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t w, u8g2_uint_t h, u8g2_uint_t r){
	ESP_LOGI(this->tag.c_str(), "DrawRFrame(%u, %u, %u, %u)", x, y, w, h, r);
	u8g2_DrawRFrame(&(this->u8g2), x, y, w, h, r);
}

void GenericLcd::SetFlipMode(uint8_t mode){
	ESP_LOGI(this->tag.c_str(), "SetFlipMode(%u)", mode);
	u8g2_SetFlipMode(&(this->u8g2), mode);
}

void GenericLcd::SetContrast(uint8_t value){
	ESP_LOGI(this->tag.c_str(), "SetContrast(%u)", value);
	u8g2_SetContrast(&(this->u8g2), value);
}

