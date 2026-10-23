#include "mt6701_spi.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "mt6701";

static uint8_t tableCRC6[64] = {
    0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09, 0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
    0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39, 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
    0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A, 0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
    0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A, 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02};

static uint8_t CRC6_43_18bit(uint32_t w)
{
    uint8_t idx = (uint8_t)((w >> 12u) & 0x3Fu);
    uint8_t b = (uint8_t)((w >> 6u) & 0x3Fu);
    idx = b ^ tableCRC6[idx];
    b = (uint8_t)(w & 0x3Fu);
    idx = b ^ tableCRC6[idx];
    return tableCRC6[idx];
}

Mt6701Spi::Mt6701Spi(spi_host_device_t host, gpio_num_t sclk, gpio_num_t miso, gpio_num_t mosi, gpio_num_t cs)
    : _host(host), _sclk(sclk), _miso(miso), _mosi(mosi), _cs(cs)
{
}

const char *Mt6701Spi::bus_name() const
{
    switch (_bus) {
    case Mt6701Bus::Spi:
        return "SPI";
    case Mt6701Bus::I2c:
        return "I2C";
    default:
        return "NONE";
    }
}

bool Mt6701Spi::init_spi()
{
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = _mosi;
    buscfg.miso_io_num = _miso;
    buscfg.sclk_io_num = _sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;

    esp_err_t ret = spi_bus_initialize(_host, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init: %s", esp_err_to_name(ret));
        return false;
    }

    spi_device_interface_config_t dev = {};
    dev.mode = 3;
    dev.clock_speed_hz = 1 * 1000 * 1000;
    dev.spics_io_num = _cs;
    dev.queue_size = 1;
    dev.cs_ena_pretrans = 2;
    dev.cs_ena_posttrans = 2;

    ret = spi_bus_add_device(_host, &dev, &_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device: %s", esp_err_to_name(ret));
        spi_bus_free(_host);
        return false;
    }

    _bus = Mt6701Bus::Spi;
    _installed = true;
    return true;
}

bool Mt6701Spi::init_i2c()
{
    gpio_reset_pin(_sclk);
    gpio_reset_pin(_miso);
    if (_cs >= 0) {
        gpio_reset_pin(_cs);
    }

    i2c_config_t cfg = {};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = _miso;
    cfg.scl_io_num = _sclk;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = 100000; /* 探测用慢速 */

    _i2c_bus = i2c_bus_create(I2C_NUM_0, &cfg);
    if (!_i2c_bus) {
        ESP_LOGE(TAG, "I2C bus create fail");
        return false;
    }
    _i2c_dev = i2c_bus_device_create(_i2c_bus, 0x06, 0);
    if (!_i2c_dev) {
        ESP_LOGE(TAG, "I2C device 0x06 create fail");
        i2c_bus_delete(&_i2c_bus);
        _i2c_bus = nullptr;
        return false;
    }

    uint8_t buf[2] = {0};
    if (i2c_bus_read_bytes(_i2c_dev, 0x03, 2, buf) != ESP_OK) {
        ESP_LOGW(TAG, "I2C 0x06 无应答");
        i2c_bus_device_delete(&_i2c_dev);
        i2c_bus_delete(&_i2c_bus);
        _i2c_dev = nullptr;
        _i2c_bus = nullptr;
        return false;
    }

    _bus = Mt6701Bus::I2c;
    _installed = true;
    _last_raw = ((uint32_t)buf[0] << 8) | buf[1];
    _last_crc_ok = true;
    ESP_LOGW(TAG, "改用 I2C 成功 raw=0x%04lX (SCL=%d SDA=%d)",
             (unsigned long)_last_raw, (int)_sclk, (int)_miso);
    return true;
}

bool Mt6701Spi::spi_probe_ok()
{
    int ok = 0;
    for (int i = 0; i < 40; i++) {
        read_spi();
        if (_last_crc_ok) {
            ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ok >= 3;
}

void Mt6701Spi::init()
{
    if (!_lock) {
        _lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(_lock != nullptr, , TAG, "mutex fail");
    }

    /* 上电先看 DO 空闲电平：悬空/开路多为 1；对地短路为 0 */
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << _miso);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io);
    int miso_idle = gpio_get_level(_miso);
    ESP_LOGI(TAG, "DO/MISO GPIO%d idle(pullup)=%d (1=正常悬空, 0=可能对地短路)", (int)_miso, miso_idle);
    gpio_reset_pin(_miso);

    ESP_LOGI(TAG, "probe SPI CS=%d CLK=%d DO=%d", (int)_cs, (int)_sclk, (int)_miso);
    if (!init_spi()) {
        ESP_LOGE(TAG, "SPI 初始化失败");
        return;
    }

    if (spi_probe_ok()) {
        ESP_LOGI(TAG, "SPI SSI OK");
        Sensor::init();
        return;
    }

    ESP_LOGW(TAG, "SPI 无有效 CRC 帧，尝试 I2C(SCL=CLK,SDA=DO)...");
    spi_bus_remove_device(_dev);
    spi_bus_free(_host);
    _dev = nullptr;
    _bus = Mt6701Bus::None;
    _installed = false;

    if (init_i2c()) {
        Sensor::init();
        return;
    }

    /* I2C 也失败：仍挂回 SPI，方便串口继续看 raw */
    ESP_LOGE(TAG, "编码器无响应。请查: 模块3V3/GND、DO→GPIO13、CLK→GPIO12、CS→GPIO10(SSI必须)");
    gpio_reset_pin(_sclk);
    gpio_reset_pin(_miso);
    if (_cs >= 0) {
        gpio_reset_pin(_cs);
    }
    if (init_spi()) {
        ESP_LOGW(TAG, "保持 SPI 诊断模式（角度不可靠直到接线正确）");
        Sensor::init();
    }
}

void Mt6701Spi::deinit()
{
    if (_lock) {
        xSemaphoreTake(_lock, portMAX_DELAY);
    }
    if (_bus == Mt6701Bus::Spi && _dev) {
        spi_bus_remove_device(_dev);
        spi_bus_free(_host);
        _dev = nullptr;
    }
    if (_bus == Mt6701Bus::I2c) {
        if (_i2c_dev) {
            i2c_bus_device_delete(&_i2c_dev);
            _i2c_dev = nullptr;
        }
        if (_i2c_bus) {
            i2c_bus_delete(&_i2c_bus);
            _i2c_bus = nullptr;
        }
    }
    _bus = Mt6701Bus::None;
    _installed = false;
    if (_lock) {
        xSemaphoreGive(_lock);
    }
}

float Mt6701Spi::read_spi()
{
    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_RXDATA;
    t.length = 24;
    t.rxlength = 24;

    esp_err_t ret = spi_device_polling_transmit(_dev, &t);
    if (ret != ESP_OK) {
        _last_crc_ok = false;
        _crc_fail++;
        return _angle;
    }

    uint32_t spi_32 = ((uint32_t)t.rx_data[0] << 16) | ((uint32_t)t.rx_data[1] << 8) | t.rx_data[2];
    _last_raw = spi_32;

    const bool stuck = (spi_32 == 0u) || (spi_32 == 0xFFFFFFu);
    uint8_t received_crc = (uint8_t)(spi_32 & 0x3Fu);
    uint8_t calculated_crc = CRC6_43_18bit(spi_32 >> 6);
    _last_crc_ok = !stuck && (received_crc == calculated_crc);

    float ang = (float)(spi_32 >> 10) * 2.0f * 3.14159265f / 16384.0f;
    if (_last_crc_ok) {
        _crc_ok++;
        _angle = ang;
    } else {
        _crc_fail++;
        if ((_crc_fail % 500) == 1) {
            ESP_LOGW(TAG, "SPI raw=0x%06lX stuck=%d (ok=%lu fail=%lu)",
                     (unsigned long)spi_32, (int)stuck,
                     (unsigned long)_crc_ok, (unsigned long)_crc_fail);
        }
    }
    return _angle;
}

float Mt6701Spi::read_i2c()
{
    uint8_t buf[2] = {0};
    if (i2c_bus_read_bytes(_i2c_dev, 0x03, 2, buf) != ESP_OK) {
        _last_crc_ok = false;
        _crc_fail++;
        return _angle;
    }
    uint16_t raw14 = (uint16_t)(((buf[0] << 6) | (buf[1] >> 2)) & 0x3FFF);
    _last_raw = raw14;
    _last_crc_ok = true;
    _crc_ok++;
    _angle = (float)raw14 * 2.0f * 3.14159265f / 16384.0f;
    return _angle;
}

float Mt6701Spi::getSensorAngle()
{
    if (!_installed) {
        return _angle;
    }
    if (_lock) {
        xSemaphoreTake(_lock, portMAX_DELAY);
    }
    float ang = _angle;
    if (_bus == Mt6701Bus::Spi) {
        ang = read_spi();
    } else if (_bus == Mt6701Bus::I2c) {
        ang = read_i2c();
    }
    if (_lock) {
        xSemaphoreGive(_lock);
    }
    return ang;
}
