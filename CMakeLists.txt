idf_component_register(SRCS "generic_lcd.cpp" "u8g2_esp32_hal.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES "u8g2"
                             "elrebo-de__i2c_master"
                             "driver")

