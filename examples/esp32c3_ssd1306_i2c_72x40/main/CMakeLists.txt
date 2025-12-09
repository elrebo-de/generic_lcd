idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES "elrebo-de__generic_lcd"
                             "u8g2"
                             "u8g2-hal-esp-idf")
