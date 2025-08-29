#ifndef _RADIOLIB_ECC1101_H

#include <RadioLib.h>
//#include <CC1101.h>

#define DEFAULT_CC1101_SPI SPIClass(HSPI)

typedef struct {
  uint32_t frequency_coarse;
  int rssi_coarse;
  uint32_t frequency_fine;
  int rssi_fine;
} FrequencyRSSI;

struct s_cc1101_rf_rx_settings {
    float freq;
    float br;
    float freqDev;
    float rxBw;
    uint8_t modulation;
};

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

  uint8_t get_rxfifo_available(void);
  uint8_t get_radio_state(void);
  int16_t setInfiniteLengthMode();
  BaseType_t set_rf(s_cc1101_rf_rx_settings *settings);
  BaseType_t scan(FrequencyRSSI *frequency_rssi, int rssi_threshold = -75);

private:
  void rx_isr_cb(void);
  void rx_task_cb(void *pv);
  TaskHandle_t rx_task;
  SPIClass *_spi;
  struct s_eCC1101_pins _pins;
  std::vector<int32_t> _cs_unused;
  StreamBufferHandle_t rxBuffer;
};
#endif /*  _RADIOLIB_ECC1101_H */

