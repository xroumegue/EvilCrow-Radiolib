#include "eCC1101.h"

eCC1101::eCC1101(struct s_eCC1101_pins& pins, SPIClass& spi, const std::vector<int32_t> cs_unused, uint32_t spiClk):
        CC1101(new Module(pins.cs, pins.gdo0, pins.rst, pins.gdo2, spi, SPISettings(spiClk, MSBFIRST, SPI_MODE0))), _spi(&spi), _pins(pins), _cs_unused(cs_unused) {
}

int16_t eCC1101::begin(float freq, float br, float freqDev, float rxBw, int8_t pwr, uint8_t preambleLength){
        char buf[128];
        /* Unselect the others SPI Slaves */ 
        for (auto it = _cs_unused.begin(); it != _cs_unused.end(); ++it) {
                snprintf(buf, 128, "Unselect pin %d\n", *it);
                Serial.print(buf);
                pinMode(*it, OUTPUT);
                digitalWrite(*it, HIGH);
        }

        _spi->end();

        /*Initialize the SPI pins for THIS device */ 
        _spi->begin(_pins.clk, _pins.miso, _pins.mosi, _pins.cs);
        snprintf(buf, 128, "SPI pins: clk: %d, miso: %d, mosi: %d, cs: %d\n", _pins.clk, _pins.miso, _pins.mosi, _pins.cs);
        Serial.print(buf);
        delay(150);
        Serial.println("Module eCC1101 Initialized ");
        return CC1101::begin(freq, br, freqDev, rxBw, pwr, preambleLength);
}
