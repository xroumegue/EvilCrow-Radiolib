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

BaseType_t eCC1101::scan(FrequencyRSSI *frequency_rssi, int rssi_threshold) {
  static const uint32_t subghz_frequency_list[] = {
      /* 300 - 348 */
      300000000,
      302757000,
      303875000,
      303900000,
      304250000,
      307000000,
      307500000,
      307800000,
      309000000,
      310000000,
      312000000,
      312100000,
      312200000,
      313000000,
      313850000,
      314000000,
      314350000,
      314980000,
      315000000,
      318000000,
      330000000,
      345000000,
      348000000,
      350000000,
  
      /* 387 - 464 */
      387000000,
      390000000,
      418000000,
      430000000,
      430500000,
      431000000,
      431500000,
      433075000, /* LPD433 first */
      433220000,
      433420000,
      433657070,
      433889000,
      433920000, /* LPD433 mid */
      434075000,
      434176948,
      434190000,
      434390000,
      434420000,
      434620000,
      434775000, /* LPD433 last channels */
      438900000,
      440175000,
      464000000,
      467750000,
  
      /* 779 - 928 */
      779000000,
      868350000,
      868400000,
      868800000,
      868950000,
      906400000,
      915000000,
      925000000,
      928000000,
  };

  int rssi;
  *frequency_rssi = {.frequency_coarse = 0,
                     .rssi_coarse = -100,
                     .frequency_fine = 0,
                     .rssi_fine = -100};

  // First stage: coarse scan
  setRxBandwidth(
      650); // 58, 68, 81, 102, 116, 135, 162, 203, 232, 270, 325, 406, 464,
  // 541, 650 and 812 kHz    (81kHz seems to work best for me)
  size_t array_size =
      sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]);
  for (size_t i = 0; i < array_size; i++) {
    uint32_t frequency = subghz_frequency_list[i];
    if (frequency != 467750000 && frequency != 464000000 &&
        frequency != 390000000 && frequency != 312000000 &&
        frequency != 312100000 && frequency != 312200000 &&
        frequency != 440175000) {
      setFrequency((float)frequency / 1000000.0);
      receiveDirect();
      delay(2);
      rssi = getRSSI();

      if (frequency_rssi->rssi_coarse < rssi) {
        frequency_rssi->rssi_coarse = rssi;
        frequency_rssi->frequency_coarse = frequency;
      }
    }
  }

  // Second stage: fine scan
  if (frequency_rssi->rssi_coarse > rssi_threshold) {
    // for example -0.3 ... 433.92 ... +0.3 step 20KHz
    setRxBandwidth(58);
    for (uint32_t i = frequency_rssi->frequency_coarse - 300000;
         i < frequency_rssi->frequency_coarse + 300000; i += 20000) {
      uint32_t frequency = i;
      setFrequency((float)frequency / 1000000.0);
      receiveDirect();
      delay(2);
      rssi = getRSSI();

      if (frequency_rssi->rssi_fine < rssi) {
        frequency_rssi->rssi_fine = rssi;
        frequency_rssi->frequency_fine = frequency;
      }
    }
  }

  return pdFALSE;
}

uint8_t eCC1101::get_rxfifo_available(void) {
  uint8_t bytesInFIFO = SPIgetRegValue(RADIOLIB_CC1101_REG_RXBYTES, 6, 0);

#if CC1101_DEBUG
  char buf[64];
  snprintf(buf, 64, "[CC1101] RXFIFO AVAILABLE: %d", bytesInFIFO);
  Serial.println(F(buf));
#endif

  return bytesInFIFO;
}

uint8_t eCC1101::get_radio_state(void) {

  int8_t radio_state = SPIgetRegValue(RADIOLIB_CC1101_REG_MARCSTATE, 4, 0);
#if CC1101_DEBUG
  char buf[64];
  snprintf(buf, 64, "[CC1101] FSM: %#x", radio_state);
  Serial.println(F(buf));
#endif

  return radio_state;
}

int16_t eCC1101::setInfiniteLengthMode()
{
    // infinite packet mode
    int16_t state = SPIsetRegValue(RADIOLIB_CC1101_REG_PKTCTRL0, RADIOLIB_CC1101_LENGTH_CONFIG_INFINITE, 1, 0);
    RADIOLIB_ASSERT(state);
    /* PKT MUST NOT BE 0 */
    size_t len = 255;
    state = SPIsetRegValue(RADIOLIB_CC1101_REG_PKTLEN, len);
    RADIOLIB_ASSERT(state);
    // no longer in a direct mode
    directModeEnabled = false;
    // update the cached values
    packetLength = len;
    packetLengthConfig = RADIOLIB_CC1101_LENGTH_CONFIG_INFINITE;

    return state;
}

BaseType_t eCC1101::set_rf(s_cc1101_rf_rx_settings *settings) {
  setFrequency(settings->freq);
  setBitRate(settings->br);
  setFrequencyDeviation(settings->freqDev);
  setRxBandwidth(settings->rxBw);
  setOOK(settings->modulation == RADIOLIB_CC1101_MOD_FORMAT_ASK_OOK);

  return pdFALSE;
}



#define TX_BIT 0x01
#define RX_BIT 0x02
void eCC1101::rx_isr_cb(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Serial.print(F("[CC1101] IRQ!\n"));
  xTaskNotifyFromISR(rx_task, RX_BIT, eSetBits, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void eCC1101::rx_task_cb(void *pv)
{
  for(;;) {
  
  }
}
