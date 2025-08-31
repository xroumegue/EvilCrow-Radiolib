#include "eCC1101.h"
#include "portmacro.h"

#define MAX(x, y) (x < y ? y : x)
#define MIN(x, y) (x < y ? x : y)

eCC1101::eCC1101(struct s_eCC1101_pins& pins, SPIClass& spi, const std::vector<int32_t> cs_unused, uint32_t spiClk):
        CC1101(new Module(pins.cs, pins.gdo0, pins.rst, pins.gdo2, spi, SPISettings(spiClk, MSBFIRST, SPI_MODE0))),
        _spi(&spi), _pins(pins), _cs_unused(cs_unused),
        _rxBufferSize(1024), _rxBufferTriggerLevel(32) {

    _rxStreamBuffer = xStreamBufferCreate(_rxBufferSize, _rxBufferTriggerLevel );

    xTaskCreate(
        _rx_thread,
        "eCC1101 RX",
        4096,
        (void *) this,
        tskRX_PRIORITY,
        &_rx_task
    );
}

void eCC1101::setPacketReceivedAction(void (*func)(void* pObj))
{
    setGdo0Action(func, this->mod->hal->GpioInterruptRising);
}

void eCC1101::setGdo0Action(void (*func)(void* pObj), uint32_t dir) {
  attachInterruptArg(this->mod->hal->pinToInterrupt(this->mod->getIrq()), func, this, dir);
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

void eCC1101::_rx_cb()
{
  const TickType_t x1000ms = pdMS_TO_TICKS(1000);
  const TickType_t x100ms = pdMS_TO_TICKS(100);
  for(;;) {
    uint32_t ulNotifiedValue = 0;
    BaseType_t xResult;
    xResult = xTaskNotifyWait(pdFALSE,          /* Don't clear bits on entry. */
                              ULONG_MAX,        /* Clear all bits on exit. */
                              &ulNotifiedValue, /* Stores the notified value. */
                              x1000ms);

#if CC1101_DEBUG
    Serial.print(F("[CC1101] Thread Wakeup!\n"));
#endif
    if (xResult == pdPASS) {
      if ((ulNotifiedValue & RX_BIT) != 0) {
        if ((ulNotifiedValue & RAW_BIT) != 0) {
            uint8_t bytesInFIFO = get_rxfifo_available();
            SPIreadRegisterBurst(RADIOLIB_CC1101_REG_FIFO, bytesInFIFO, _rxFifo);
            size_t bytesSent = xStreamBufferSend(_rxStreamBuffer,
                          _rxFifo,
                          bytesInFIFO,
                          x100ms);
#if CC1101_DEBUG
            char buf[64];
            snprintf(buf, sizeof(buf), "%d/%d\n", bytesSent, bytesInFIFO);
            Serial.print(buf);
#endif
            configASSERT(bytesSent == bytesInFIFO);
        }
        if ((ulNotifiedValue & PKT_BIT) != 0) {
        //    uint8_t lbuf[pktLen];
        //    readData(lbuf, pktLen);
        //    size_t len = MIN(pktLen, remaining);
        //    memcpy(data, lbuf, len);
        //    data += len;
        //    remaining -= len;
        }
      }
    } 
  }
}

int16_t eCC1101::startRawReceive(struct s_cc1101_rf_rx_settings *settings) {
  setPromiscuousMode(true, false);
  setPacketReceivedAction(eCC1101::_rx_isr_cb);
  SPIsetRegValue(RADIOLIB_CC1101_REG_PKTCTRL1, RADIOLIB_CC1101_APPEND_STATUS_OFF, 3, 3);
  set_rf(settings);
  disableAddressFiltering();
  SPIsetRegValue(RADIOLIB_CC1101_REG_MCSM1, RADIOLIB_CC1101_RXOFF_RX, 3, 2);
  setInfiniteLengthMode();

  startReceive();

  return 0;
}

int16_t eCC1101::rawReceive(uint8_t *data, size_t len, TickType_t xTicksToWait) {
    ssize_t remaining = len;
    TickType_t startTime = xTaskGetTickCount();

#if CC1101_DEBUG
    char buf[64];
    snprintf(buf, sizeof(buf), "RX: %u\n", len);
    Serial.print(buf);
#endif

    while (remaining > 0) {
        size_t received = xStreamBufferReceive( _rxStreamBuffer,
                             data,
                             remaining,
                             xTicksToWait);
#if CC1101_DEBUG
    char buf[64];
    snprintf(buf, sizeof(buf), "RX: %u %u %u\n", len, received, remaining);
    Serial.print(buf);
#endif

        TickType_t endTime = xTaskGetTickCount();

        if (received < 0)
            break;
        data += received;
        remaining -= received;
        if ((endTime - startTime) > xTicksToWait) {
            break;
        }
    }

    return (len - remaining);
}

int16_t eCC1101::stopRawReceive() {
  clearPacketReceivedAction();
  SPIsendCommand(RADIOLIB_CC1101_CMD_FLUSH_RX);
  setPromiscuousMode(false, false);
  return 0;
}

#if 0
static BaseType_t cc1101_receiveStream(eCC1101 *cc1101, unsigned char *data, size_t len) {
  int state;

  state = cc1101->startReceive();
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(5000);
  size_t remaining = len;
  while (remaining) {
    uint32_t ulNotifiedValue = 0;
    BaseType_t xResult;
    xResult = xTaskNotifyWait(pdFALSE,          /* Don't clear bits on entry. */
                              ULONG_MAX,        /* Clear all bits on exit. */
                              &ulNotifiedValue, /* Stores the notified value. */
                              xMaxBlockTime);

    if (xResult == pdPASS) {
      if ((ulNotifiedValue & RX_BIT) != 0) {
#if 1
        uint8_t bytesInFIFO =
            MIN(cc1101->get_rxfifo_available(), remaining);
        cc1101->SPIreadRegisterBurst(RADIOLIB_CC1101_REG_FIFO, bytesInFIFO,
                                     data);
        data += bytesInFIFO;
        remaining -= bytesInFIFO;
#else
        uint8_t lbuf[pktLen];
        cc1101->readData(lbuf, pktLen);
        size_t len = MIN(pktLen, remaining);
        memcpy(data, lbuf, len);
        data += len;
        remaining -= len;

#endif
      }
    } else {
      break;
    }
  }
  cc1101->standby();
  return len - remaining;
}

static BaseType_t c1101_receive(eCC1101 *cc1101, s_cc1101_rf_rx_settings *settings, unsigned char *data, size_t len) {
  xHandlingTask = xTaskGetCurrentTaskHandle();

  int state;

  cc1101->setPromiscuousMode(true, false);
  cc1101->setPacketReceivedAction(_rx_isr_cb);
  state = cc1101->SPIsetRegValue(RADIOLIB_CC1101_REG_PKTCTRL1,
                                 RADIOLIB_CC1101_APPEND_STATUS_OFF, 3, 3);

  state = cc1101->set_rf(settings);
  state = cc1101->disableAddressFiltering();
  cc1101->SPIsetRegValue(RADIOLIB_CC1101_REG_MCSM1, RADIOLIB_CC1101_RXOFF_RX, 3,
                         2);
  cc1101->setInfiniteLengthMode();

  size_t received = cc1101_receiveStream(cc1101, data, len);
  cc1101->clearPacketReceivedAction();
  cc1101->SPIsendCommand(RADIOLIB_CC1101_CMD_FLUSH_RX);
  cc1101->setPromiscuousMode(false, false);

  return received;
}

#endif
