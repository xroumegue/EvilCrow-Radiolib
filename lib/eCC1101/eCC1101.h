#ifndef _RADIOLIB_ECC1101_H

#include <RadioLib.h>
//#include <CC1101.h>

#define RX_BIT  BIT(0)
#define TX_BIT  BIT(1)
#define PKT_BIT BIT(6)
#define RAW_BIT BIT(7)

#define tskRX_PRIORITY (configMAX_PRIORITIES - 10)

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
  int16_t setInfiniteLengthMode(void);
  BaseType_t set_rf(s_cc1101_rf_rx_settings *settings);
  BaseType_t scan(FrequencyRSSI *frequency_rssi, int rssi_threshold = -75);
  int16_t startRawReceive(struct s_cc1101_rf_rx_settings *settings);
  int16_t stopRawReceive(void);
  int16_t rawReceive(uint8_t *data, size_t len, TickType_t xTicksToWait = pdMS_TO_TICKS(5000));
  void setPacketReceivedAction(void (*isr)(void*pObj));
  void setGdo0Action(void (*func)(void* pObj), uint32_t dir);
  TaskHandle_t get_rx_task() {
    return _rx_task;
  }

private:
  uint8_t _rxFifo[64];
  static void _rx_thread(void *pv) {
    eCC1101 *instance = static_cast<eCC1101*>(pv);
    instance->_rx_cb();
  }

  static void _rx_isr_cb(void *pObj) {
    eCC1101 *instance = static_cast<eCC1101*>(pObj);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#if CC1101_DEBUG
    Serial.print(F("[CC1101] IRQ!\n"));
#endif
    xTaskNotifyFromISR(instance->get_rx_task(), RX_BIT | RAW_BIT, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }


  void _rx_cb();
  TaskHandle_t _rx_task;
  size_t _rxBufferSize;
  size_t _rxBufferTriggerLevel;
  StreamBufferHandle_t _rxStreamBuffer;
  SPIClass *_spi;
  struct s_eCC1101_pins _pins;
  std::vector<int32_t> _cs_unused;
};
#endif /*  _RADIOLIB_ECC1101_H */

