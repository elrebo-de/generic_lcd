idf_component_register(SRCS "generic_lcd.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES "u8g2"
                             "u8g2_hal_espidf"
                             "elrebo-de__i2c_master"
                             "driver")

