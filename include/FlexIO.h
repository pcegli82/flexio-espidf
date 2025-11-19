#pragma once

#include <stdint.h>
#include <stdbool.h>

extern "C" {
#include "driver/i2c.h"
}

/*****************************************************************
 *  FlexIO – PCAL6416-Treiber mit Open-Drain-Ausgängen
 *  true  = Ausgang   (Port-1, Open-Drain, High per Pull-Up)
 *  false = Eingang   (Port-0, Interrupt-fähig)
 *****************************************************************/
class FlexIO
{
public:
    FlexIO();

    /**
     * @param sclPin   GPIO des SCL
     * @param sdaPin   GPIO des SDA
     * @param intPin   GPIO des INT-Pins (nur gemerkt, keine IRQ-Logik hier)
     * @param flexIOConfig  8-Array: true = Ausgang, false = Eingang
     * @param i2cAddr  I2C-Adresse des PCAL6416
     * @param port     I2C-Port (I2C_NUM_0 oder I2C_NUM_1)
     */
    bool begin(int sclPin, int sdaPin, int intPin,
               const bool flexIOConfig[8],
               uint8_t i2cAddr = 0x20,
               i2c_port_t port = I2C_NUM_0);

    // 0 = OK, -1 = Kanal ist Eingang, -2 = ungültig, -3 = I²C-Fehler
    int  Write(uint8_t channel, bool state);
    int  Read(uint8_t channel);

private:
    int        _sclPin, _sdaPin, _intPin;
    uint8_t    _i2cAddr;
    i2c_port_t _port;
    bool       _flexIO_config[8];

    /* ------ low-level Helfer ---------------------------------- */
    esp_err_t readReg8  (uint8_t reg, uint8_t *value);
    esp_err_t writeReg8 (uint8_t reg, uint8_t value);

    /* ------ Initialisierung ----------------------------------- */
    void configurePCAL6416();
    void configureInterruptMask();
};
