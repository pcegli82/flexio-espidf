#include "FlexIO.h"

extern "C" {
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

static const char *TAG = "FlexIO";

/* ---------- Konstruktor ------------------------------------- */
FlexIO::FlexIO()
    : _sclPin(-1),
      _sdaPin(-1),
      _intPin(-1),
      _i2cAddr(0x20),
      _port(I2C_NUM_0),
      _intPending(false),
      _lastInput0(0)
{
    for (int i = 0; i < 8; ++i) {
        _flexIO_config[i] = false;
    }
}

/* ---------- Public: begin() --------------------------------- */
bool FlexIO::begin(int sclPin, int sdaPin, int intPin,
                   const bool flexIOConfig[8],
                   uint8_t i2cAddr,
                   i2c_port_t port)
{
    _sclPin  = sclPin;
    _sdaPin  = sdaPin;
    _intPin  = intPin;
    _i2cAddr = i2cAddr;
    _port    = port;

    for (int i = 0; i < 8; ++i) {
        _flexIO_config[i] = flexIOConfig[i];
    }

    // I2C-Master konfigurieren (100 kHz wie Arduino Wire)
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num    = (gpio_num_t)_sdaPin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num    = (gpio_num_t)_sclPin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // 100 kHz

    esp_err_t err = i2c_param_config(_port, &conf);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = i2c_driver_install(_port, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }

    // PCAL konfigurieren
    err = configurePCAL6416();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "configurePCAL6416() failed: %s", esp_err_to_name(err));
        return false;
    }

    // Interrupt-Maske im PCAL setzen
    err = configureInterruptMask();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "configureInterruptMask() failed: %s", esp_err_to_name(err));
        return false;
    }

    // Optional INT-GPIO auf ESP32 konfigurieren
    if (_intPin >= 0) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << _intPin);
        io_conf.mode         = GPIO_MODE_INPUT;
        io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;      // INT ist open-drain, aktiv LOW
        io_conf.intr_type    = GPIO_INTR_NEGEDGE;       // fallende Flanke reicht

        err = gpio_config(&io_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "gpio_config(INT) failed: %s", esp_err_to_name(err));
            return false;
        }

        // ISR-Service installieren (falls noch nicht vorhanden)
        esp_err_t isrErr = gpio_install_isr_service(0);
        if (isrErr != ESP_OK && isrErr != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(isrErr));
            return false;
        }

        // Unsere ISR registrieren
        isrErr = gpio_isr_handler_add((gpio_num_t)_intPin, &FlexIO::isrThunk, this);
        if (isrErr != ESP_OK) {
            ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(isrErr));
            return false;
        }
    }

    // Start-Zustand der Inputs einmal einlesen (optional)
    uint8_t dummy = 0;
    if (readReg8(0x00, &dummy) == ESP_OK) {
        _lastInput0 = dummy;
    }

    ESP_LOGI(TAG, "FlexIO init OK (addr=0x%02X, port=%d, intPin=%d)",
             _i2cAddr, (int)_port, _intPin);
    return true;
}

/* ---------- Low-Level-Helfer -------------------------------- */
/*  readReg8: exakt wie Arduino:
 *    1) WRITE (Reg-Adresse) + STOP
 *    2) READ  (1 Byte)      + STOP
 */
esp_err_t FlexIO::readReg8(uint8_t reg, uint8_t *value)
{
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;

    // 1) Registerpointer setzen (WRITE + STOP)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
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

    // 2) Wert lesen (wie Wire.requestFrom)
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
esp_err_t FlexIO::configurePCAL6416()
{
    esp_err_t err;

    /* 1) Latches erst auf LOW (0) setzen ----------------------- */
    // OUT0 + OUT1
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x02, true);     // OUT0-Registeradresse
    i2c_master_write_byte(cmd, 0x00, true);     // OUT0  (Port-0) = 0
    i2c_master_write_byte(cmd, 0x00, true);     // OUT1  (Port-1) = 0
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Init OUT0/OUT1 failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 2) IOCON0/1: Open-Drain aktivieren (falls vorhanden) ----- */
    // Manche Derivate haben 0x0A/0x0B evtl. nicht -> NICHT fatal
    uint8_t iocon0 = 0, iocon1 = 0;
    err = readReg8(0x0A, &iocon0);
    if (err == ESP_OK) {
        esp_err_t err2 = readReg8(0x0B, &iocon1);
        if (err2 == ESP_OK) {
            iocon0 |= 0b00000010; // OD auf Port-0 (falls relevant)
            iocon1 |= 0b00000010; // OD auf Port-1
            writeReg8(0x0A, iocon0);
            writeReg8(0x0B, iocon1);
        } else {
            ESP_LOGW(TAG, "readReg8(IOCON1 0x0B) failed, skipping OD-bit: %s",
                     esp_err_to_name(err2));
        }
    } else {
        ESP_LOGW(TAG, "readReg8(IOCON0 0x0A) failed, skipping OD-bits: %s",
                 esp_err_to_name(err));
        // NICHT abbrechen – restliche Konfiguration funktioniert trotzdem
    }

    /* 3) Richtung einstellen (0 = Out, 1 = In) ----------------- */
    uint8_t port0_cfg = 0;
    uint8_t port1_cfg = 0;
    for (int i = 0; i < 8; ++i) {
        if (!_flexIO_config[i]) {           // false -> Eingang
            port0_cfg |= (uint8_t)(1 << i);
            port1_cfg |= (uint8_t)(1 << i);
        }
    }

    err = writeReg8(0x06, port0_cfg);   // CFG0
    if (err != ESP_OK) return err;
    err = writeReg8(0x07, port1_cfg);   // CFG1
    if (err != ESP_OK) return err;

    /* 4) Pull-Up-Richtung für Ausgänge vormerken --------------- */
    uint8_t pullSel1 = 0;
    for (int i = 0; i < 8; ++i) {
        if (_flexIO_config[i]) pullSel1 |= (uint8_t)(1 << i);  // Ausgang = Up
    }
    err = writeReg8(0x48, 0x00);      // PULL_SEL0 (Eingänge → keine Pulls)
    if (err != ESP_OK) return err;
    err = writeReg8(0x49, pullSel1);  // PULL_SEL1 (Ausgänge → Up)
    if (err != ESP_OK) return err;
    err = writeReg8(0x46, 0x00);      // PULL_EN0
    if (err != ESP_OK) return err;
    err = writeReg8(0x47, 0x00);      // PULL_EN1 (start: alles aus)
    if (err != ESP_OK) return err;

    return ESP_OK;
}

/* ---------- Interrupt-Maske --------------------------------- */
esp_err_t FlexIO::configureInterruptMask()
{
    // 1 = maskiert, 0 = Interrupt aktiv
    uint8_t port0_mask = 0xFF;
    uint8_t port1_mask = 0xFF;  // Ausgänge bleiben maskiert

    for (int i = 0; i < 8; ++i) {
        if (!_flexIO_config[i]) {           // false -> Eingang (Port-0)
            port0_mask &= (uint8_t)~(1 << i);  // unmask
        }
    }

    esp_err_t err = writeReg8(0x4A, port0_mask); // INT_MASK0
    if (err != ESP_OK) return err;

    err = writeReg8(0x4B, port1_mask);          // INT_MASK1
    if (err != ESP_OK) return err;

    return ESP_OK;
}

/* ---------- Write() – Open-Drain-Logik ---------------------- */
int FlexIO::Write(uint8_t channel, bool state)
{
    if (channel < 1 || channel > 8) return -2;
    uint8_t idx  = (uint8_t)(channel - 1);
    uint8_t mask = (uint8_t)(1 << idx);

    if (!_flexIO_config[idx]) return -1;  // Kanal ist Eingang

    uint8_t outLatch = 0;
    uint8_t pullEn   = 0;

    if (readReg8(0x03, &outLatch) != ESP_OK) return -3;  // Port-1 Out-Latch
    if (readReg8(0x47, &pullEn)   != ESP_OK) return -3;  // PULL_EN1

    if (state) {            /* -------- HIGH ------------------- */
        outLatch |= mask;                     // Hi-Z
        pullEn   |= mask;                     // Pull-Up EIN
    } else {                /* -------- LOW -------------------- */
        outLatch &= (uint8_t)~mask;           // aktiv LOW
        pullEn   &= (uint8_t)~mask;           // Pull-Up AUS
    }

    if (writeReg8(0x03, outLatch) != ESP_OK) return -3;
    if (writeReg8(0x47, pullEn)   != ESP_OK) return -3;
    return 0;
}

/* ---------- Read() ------------------------------------------ */
int FlexIO::Read(uint8_t channel)
{
    if (channel < 1 || channel > 8) return -2;
    uint8_t idx = (uint8_t)(channel - 1);

    if (_flexIO_config[idx]) return -1;     // Ausgang

    uint8_t inVal = 0;
    if (readReg8(0x00, &inVal) != ESP_OK)  // Port-0 Input
        return -3;

    _lastInput0 = inVal;  // Cache (optional)
    return (inVal & (uint8_t)(1 << idx)) ? 1 : 0;
}

/* ---------- Interrupt-ISR ----------------------------------- */
void IRAM_ATTR FlexIO::isrThunk(void *arg)
{
    auto *self = static_cast<FlexIO*>(arg);
    if (self) {
        self->onGpioInterrupt();
    }
}

void IRAM_ATTR FlexIO::onGpioInterrupt()
{
    // Nur Flag setzen – KEINE I²C-Calls, KEIN Logging!
    _intPending = true;
}

/* ---------- processInterrupts() ------------------------------ */
void FlexIO::processInterrupts()
{
    if (_intPin < 0) return;       // kein INT-GPIO konfiguriert
    if (!_intPending) return;      // nichts anhängig

    // Flag früh löschen; bei Problemen wird es durch neue Flanke wieder gesetzt
    _intPending = false;

    // Einfach Port-0 lesen – bei den PCAL-Bausteinen reicht typischerweise
    // das Lesen der Input/Status-Register, um den INT zu quittieren.
    uint8_t in0 = 0;
    esp_err_t err = readReg8(0x00, &in0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "processInterrupts(): readReg8(0x00) failed: %s",
                 esp_err_to_name(err));
        return;
    }

    _lastInput0 = in0;

    // Optionales Debug – NICHT zu spammen
    ESP_LOGD(TAG, "PCAL INT handled, input0=0x%02X", in0);
}
