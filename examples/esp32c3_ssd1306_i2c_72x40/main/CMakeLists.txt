idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES "elrebo-de__generic_lcd"
                             "elrebo-de__i2c_master"
                             "u8g2"
                             "u8g2-hal-esp-idf")
