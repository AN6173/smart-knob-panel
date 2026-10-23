#pragma once

/**
 * MT6701 角度传感器：优先 SSI/SPI，失败则自动改 I2C（同脚 SCL/SDA）。
 */

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "i2c_bus.h"
#include "common/base_classes/Sensor.h"

enum class Mt6701Bus : uint8_t {
    None = 0,
    Spi,
    I2c,
};

class Mt6701Spi : public Sensor {
public:
    Mt6701Spi(spi_host_device_t host, gpio_num_t sclk, gpio_num_t miso, gpio_num_t mosi, gpio_num_t cs);

    void init() override;
    void deinit();
    float getSensorAngle() override;

    float cached_angle() const { return _angle; }
    uint32_t crc_ok_count() const { return _crc_ok; }
    uint32_t crc_fail_count() const { return _crc_fail; }
    uint32_t last_raw() const { return _last_raw; }
    bool last_crc_ok() const { return _last_crc_ok; }
    Mt6701Bus bus() const { return _bus; }
    const char *bus_name() const;

private:
    bool init_spi();
    bool init_i2c();
    float read_spi();
    float read_i2c();
    bool spi_probe_ok();

    spi_host_device_t _host;
    gpio_num_t _sclk;
    gpio_num_t _miso;
    gpio_num_t _mosi;
    gpio_num_t _cs;

    Mt6701Bus _bus = Mt6701Bus::None;
    spi_device_handle_t _dev = nullptr;
    i2c_bus_handle_t _i2c_bus = nullptr;
    i2c_bus_device_handle_t _i2c_dev = nullptr;
    SemaphoreHandle_t _lock = nullptr;
    bool _installed = false;

    float _angle = 0.0f;
    uint32_t _last_raw = 0;
    bool _last_crc_ok = false;
    uint32_t _crc_ok = 0;
    uint32_t _crc_fail = 0;
};
