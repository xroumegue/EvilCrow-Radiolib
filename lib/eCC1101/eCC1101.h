#ifndef _RADIOLIB_ECC1101_H

#include <RadioLib.h>
//#include <CC1101.h>

#define DEFAULT_CC1101_SPI SPIClass(HSPI)

class eCC1101: public CC1101 {
public:
  struct s_eCC1101_pins {
    uint32_t cs;
    uint32_t rst;
    uint32_t clk;
    uint32_t miso;
    uint32_t mosi;
    uint32_t gdo0;
    uint32_t gdo2;
  };

  eCC1101(struct s_eCC1101_pins& pins, SPIClass& spi, const std::vector<int32_t> cs_unused = std::vector<int32_t>(), uint32_t spiClk = 1000000);
  int16_t begin(
    float freq = RADIOLIB_CC1101_DEFAULT_FREQ,
    float br = RADIOLIB_CC1101_DEFAULT_BR,
    float freqDev = RADIOLIB_CC1101_DEFAULT_FREQDEV,
    float rxBw = RADIOLIB_CC1101_DEFAULT_RXBW,
    int8_t pwr = RADIOLIB_CC1101_DEFAULT_POWER,
    uint8_t preambleLength = RADIOLIB_CC1101_DEFAULT_PREAMBLELEN);

private:
  SPIClass *_spi;
  struct s_eCC1101_pins _pins;
  std::vector<int32_t> _cs_unused;

};
#endif /*  _RADIOLIB_ECC1101_H */

