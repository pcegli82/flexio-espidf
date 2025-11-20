#pragma once

#include <stdint.h>
#include <stdbool.h>

extern "C" {
#include "driver/i2c.h"
#include "esp_err.h"
}

/*****************************************************************
 *  FlexIO – PCAL6416-Treiber mit Open-Drain-Ausgängen
 *  true  = Ausgang   (Port-1, Open-Drain, High per Pull-Up)
 *  false = Eingang   (Port-0, Interrupt-fähig)
 *
 *  WICHTIG:
 *  - Der INT-Pin des PCAL wird als GPIO-Interrupt verwendet.
 *  - Die ISR macht nur ein Flag (_intPending = true).
 *  - I²C & Logging nur im Task-Kontext (processInterrupts()).
 *****************************************************************/
class FlexIO
{
public:
    FlexIO();

    /**
     * @param sclPin   GPIO des SCL
     * @param sdaPin   GPIO des SDA
     * @param intPin   GPIO des INT-Pins des PCAL (LOW-aktiv, open-drain).
     *                 <0 = kein Interrupt-GPIO verwenden.
     * @param flexIOConfig  8-Array: true = Ausgang, false = Eingang
     * @param i2cAddr  I2C-Adresse des PCAL6416
     * @param port     I2C-Port (I2C_NUM_0 oder I2C_NUM_1)
     */
    bool begin(int sclPin, int sdaPin, int intPin,
               const bool flexIOConfig[8],
               uint8_t i2cAddr = 0x20,
               i2c_port_t port = I2C_NUM_0);

    // 0 = OK, -1 = Kanal ist Eingang, -2 = ungültiger Kanal, -3 = I²C-Fehler
    int  Write(uint8_t channel, bool state);

    // 0/1 = Pegel, -1 = Kanal ist Ausgang, -2 = ungültiger Kanal, -3 = I²C-Fehler
    int  Read(uint8_t channel);

    /**
     * Muss regelmäßig im Task-Kontext aufgerufen werden (z.B. im Hauptloop),
     * wenn ein INT-Pin gesetzt wurde.
     *
     * Macht:
     *  - Prüft, ob ein Interrupt vom PCAL anliegt (_intPending).
     *  - Liest mindestens den Input-Port (0x00), um den PCAL-INT zu quittieren.
     *  - Optional Logging.
     *
     * Wenn kein INT-Pin übergeben wurde, ist der Call billig (so gut wie no-op).
     */
    void processInterrupts();

private:
    int        _sclPin;
    int        _sdaPin;
    int        _intPin;     // ESP32-GPIO für INT (oder <0 = aus)
    uint8_t    _i2cAddr;
    i2c_port_t _port;
    bool       _flexIO_config[8];

    volatile bool _intPending;   // wird nur im ISR gesetzt/gelöscht

    // Optionaler Cache für Inputs (Port-0); derzeit nur für Debug genutzt
    uint8_t    _lastInput0;

    /* ------ low-level Helfer ---------------------------------- */
    esp_err_t readReg8  (uint8_t reg, uint8_t *value);
    esp_err_t writeReg8 (uint8_t reg, uint8_t value);

    /* ------ Initialisierung ----------------------------------- */
    esp_err_t configurePCAL6416();
    esp_err_t configureInterruptMask();

    /* ------ Interrupt-Handling (GPIO-ISR) --------------------- */
    static void IRAM_ATTR isrThunk(void *arg);
    void        IRAM_ATTR onGpioInterrupt();
};
