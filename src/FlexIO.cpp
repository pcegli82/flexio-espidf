#include "FlexIO.h"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

static const char *TAG = "FlexIO";

/* ---------- Konstruktor ------------------------------------- */
FlexIO::FlexIO()
    : _sclPin(-1), _sdaPin(-1), _intPin(-1),
      _i2cAddr(0x20), _port(I2C_NUM_0)
{
    for (int i = 0; i < 8; ++i) _flexIO_config[i] = false;
}

/* ---------- Public: begin() --------------------------------- */
bool FlexIO::begin(int sclPin, int sdaPin, int intPin,
                   const bool flexIOConfig[8],
                   uint8_t i2cAddr,
                   conf.master.clk_speed = 100000;   // statt 400000
                   i2c_port_t port)
{
    _sclPin  = sclPin;
    _sdaPin  = sdaPin;
    _intPin  = intPin;
    _i2cAddr = i2cAddr;
    _port    = port;

    for (int i = 0; i < 8; ++i)
        _flexIO_config[i] = flexIOConfig[i];

    // I2C-Master konfigurieren
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)_sdaPin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)_sclPin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000; // 400 kHz

    esp_err_t err = i2c_param_config(_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = i2c_driver_install(_port, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }

    configurePCAL6416();
    configureInterruptMask();
    return true;
}

/* ---------- Low-Level-Helfer -------------------------------- */

esp_err_t FlexIO::readReg8(uint8_t reg, uint8_t *value)
{
    if (!value) return ESP_ERR_INVALID_ARG;

    esp_err_t err;
    i2c_cmd_handle_t cmd;

    // 1) Registerpointer setzen (WRITE + STOP)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "readReg8(0x%02X) set pointer failed: %s",
                 reg, esp_err_to_name(err));
        return err;
    }

    // 2) Registerwert lesen (neue Transaktion, wie Wire.requestFrom)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "readReg8(0x%02X) read failed: %s",
                 reg, esp_err_to_name(err));
    }
    return err;
}

esp_err_t FlexIO::writeReg8(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "writeReg8(0x%02X) failed: %s", reg, esp_err_to_name(err));
    }
    return err;
}

/* ---------- PCAL6416-Setup ---------------------------------- */
void FlexIO::configurePCAL6416()
{
    /* 1) Latches erst auf LOW (0) setzen ----------------------- */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x02, true);     // OUT0 + OUT1
    i2c_master_write_byte(cmd, 0x00, true);     // Port-0  (wird gleich Input)
    i2c_master_write_byte(cmd, 0x00, true);     // Port-1  (Outputs = LOW)
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);

    /* 2) IOCON0/1: Open-Drain aktivieren ----------------------- */
    uint8_t iocon0 = 0, iocon1 = 0;
    readReg8(0x0A, &iocon0);
    readReg8(0x0B, &iocon1);
    iocon0 |= 0b00000010;
    iocon1 |= 0b00000010;
    writeReg8(0x0A, iocon0);
    writeReg8(0x0B, iocon1);

    /* 3) Richtung einstellen (0 = Out, 1 = In) ----------------- */
    uint8_t port0_cfg = 0, port1_cfg = 0;
    for (int i = 0; i < 8; ++i) {
        if (!_flexIO_config[i]) {           // false -> Eingang
            port0_cfg |= (1 << i);
            port1_cfg |= (1 << i);
        }
    }
    writeReg8(0x06, port0_cfg);           // CFG0
    writeReg8(0x07, port1_cfg);           // CFG1

    /* 4) Pull-Up-Richtung für Ausgänge vormerken --------------- */
    uint8_t pullSel1 = 0;
    for (int i = 0; i < 8; ++i) {
        if (_flexIO_config[i]) pullSel1 |= (1 << i);  // Ausgang = Up
    }
    writeReg8(0x48, 0x00);     // PULL_SEL0 (Eingänge → keine Pulls)
    writeReg8(0x49, pullSel1);
    writeReg8(0x46, 0x00);     // PULL_EN0
    writeReg8(0x47, 0x00);     // PULL_EN1 (start: alles aus)
}

/* ---------- Interrupt-Maske --------------------------------- */
void FlexIO::configureInterruptMask()
{
    uint8_t port0_mask = 0xFF, port1_mask = 0xFF;
    for (int i = 0; i < 8; ++i) {
        if (!_flexIO_config[i]) {           // false -> Eingang
            port0_mask &= ~(1 << i);        // unmask
        }
    }
    writeReg8(0x4A, port0_mask);
    writeReg8(0x4B, port1_mask);            // Ausgänge = maskiert
}

/* ---------- Write() – Open-Drain-Logik ---------------------- */
int FlexIO::Write(uint8_t channel, bool state)
{
    if (channel < 1 || channel > 8) return -2;
    uint8_t idx  = channel - 1;
    uint8_t mask = 1 << idx;

    if (!_flexIO_config[idx]) return -1;  // Kanal ist Eingang

    uint8_t outLatch = 0, pullEn = 0;
    if (readReg8(0x03, &outLatch) != ESP_OK) return -3;  // Port-1-Latch
    if (readReg8(0x47, &pullEn)   != ESP_OK) return -3;  // PULL_EN1

    if (state) {            /* -------- HIGH ------------------- */
        outLatch |= mask;                    // Hi-Z
        pullEn   |= mask;                    // Pull-Up EIN
    } else {               /* -------- LOW -------------------- */
        outLatch &= (uint8_t)~mask;          // aktiv LOW
        pullEn   &= (uint8_t)~mask;          // Pull-Up AUS
    }

    if (writeReg8(0x03, outLatch) != ESP_OK) return -3;
    if (writeReg8(0x47, pullEn)   != ESP_OK) return -3;
    return 0;
}

/* ---------- Read() ------------------------------------------ */
int FlexIO::Read(uint8_t channel)
{
    if (channel < 1 || channel > 8) return -2;
    uint8_t idx = channel - 1;
    if (_flexIO_config[idx]) return -1;     // Ausgang

    uint8_t inVal = 0;
    if (readReg8(0x00, &inVal) != ESP_OK)  // Port-0 Input
        return -3;

    return (inVal & (1 << idx)) ? 1 : 0;
}
