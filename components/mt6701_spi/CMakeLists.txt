idf_component_register(
    SRCS "mt6701_spi.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer espressif__i2c_bus
    PRIV_REQUIRES espressif__esp_simplefoc
)
