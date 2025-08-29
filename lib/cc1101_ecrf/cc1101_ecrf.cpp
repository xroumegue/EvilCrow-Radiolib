#include <Arduino.h>
#include <FreeRTOS_CLI.h>
#include <FreeRTOS_Shell.h>
#include <RadioLib.h>
#include <eCC1101.h>
#include <SPI.h>

#define CC1101_MOD_SCK 14
#define CC1101_MOD_MISO 12
#define CC1101_MOD_MOSI 13

#define CC1101_MOD1_CSN 5
#define CC1101_MOD1_GDO0 2
#define CC1101_MOD1_GDO2 4
#define CC1101_MOD1_IRQ CC1101_MOD1_GDO0
#define CC1101_MOD1_GPIO RADIOLIB_NC

#define CC1101_MOD2_CSN 27
#define CC1101_MOD2_GDO0 25
#define CC1101_MOD2_GDO2 26
#define CC1101_MOD2_IRQ CC1101_MOD2_GDO0
#define CC1101_MOD2_GPIO RADIOLIB_NC

static SPIClass spi = SPIClass(HSPI);

static struct eCC1101::s_eCC1101_pins cc1101_mod1_pins = {
  .cs = CC1101_MOD1_CSN,
  .rst = RADIOLIB_NC,
  .clk = CC1101_MOD_SCK,
  .miso = CC1101_MOD_MISO,
  .mosi = CC1101_MOD_MOSI,
  .gdo0 = CC1101_MOD1_GDO0, /* IRQ */
  .gdo2 = RADIOLIB_NC       /* GPIO */
};

static struct eCC1101::s_eCC1101_pins cc1101_mod2_pins = {
  .cs = CC1101_MOD2_CSN,
  .rst = RADIOLIB_NC,
  .clk = CC1101_MOD_SCK,
  .miso = CC1101_MOD_MISO,
  .mosi = CC1101_MOD_MOSI,
  .gdo0 = CC1101_MOD2_GDO0, /* IRQ */
  .gdo2 = RADIOLIB_NC       /* GPIO */
};

static const std::vector<int32_t> cs_unused_mod1 = {CC1101_MOD2_CSN};
static const std::vector<int32_t> cs_unused_mod2 = {CC1101_MOD1_CSN};

#ifndef SPI_CLK_FREQ
#define SPI_CLK_FREQ 1000000
#endif

#define MAX(x, y) (x < y ? y : x)
#define MIN(x, y) (x < y ? x : y)

static eCC1101 ecrf_radios[] = {
    eCC1101(cc1101_mod1_pins, spi, cs_unused_mod1, SPI_CLK_FREQ),
    eCC1101(cc1101_mod2_pins, spi, cs_unused_mod2, SPI_CLK_FREQ),
};

static unsigned char dataBuff[512];
static unsigned char *rxPtr = dataBuff;
static unsigned char *wrPtr = dataBuff;
static size_t rxReceived = 0;
static size_t rxLength;
static eCC1101 *pCC1101 = NULL;

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

typedef struct {
  uint32_t frequency_coarse;
  int rssi_coarse;
  uint32_t frequency_fine;
  int rssi_fine;
} FrequencyRSSI;

const int rssi_threshold = -75;

uint8_t cc1101_get_radio_state(eCC1101 *cc1101) {

  int8_t radio_state =
      cc1101->SPIgetRegValue(RADIOLIB_CC1101_REG_MARCSTATE, 4, 0);
#if CC1101_DEBUG
  char buf[64];
  snprintf(buf, 64, "[CC1101] FSM: %#x", radio_state);
  Serial.println(F(buf));
#endif

  return radio_state;
}

uint8_t cc1101_get_rxfifo_available(eCC1101 *cc1101) {
  uint8_t bytesInFIFO =
      cc1101->SPIgetRegValue(RADIOLIB_CC1101_REG_RXBYTES, 6, 0);

#if CC1101_DEBUG
  char buf[64];
  snprintf(buf, 64, "[CC1101] RXFIFO AVAILABLE: %d", bytesInFIFO);
  Serial.println(F(buf));
#endif

  return bytesInFIFO;
}

static eCC1101 *cc1101_init(int id) {

  if ((id < 0) || (id > 1)) {
    Serial.print(F("[E] [CC1101] Wrong module id ... "));
    return NULL;
  }

  eCC1101 *cc1101 = &ecrf_radios[id];
  cc1101->begin();

  cc1101_get_radio_state(cc1101);
  return cc1101;
}

static int16_t cc1101_setInfiniteLengthMode(eCC1101 *cc1101)
{
    // infinite packet mode
    int16_t state = cc1101->SPIsetRegValue(RADIOLIB_CC1101_REG_PKTCTRL0, RADIOLIB_CC1101_LENGTH_CONFIG_INFINITE, 1, 0);
    RADIOLIB_ASSERT(state);
    /* PKT MUST NOT BE 0 */
    size_t len = 255;
    state = cc1101->SPIsetRegValue(RADIOLIB_CC1101_REG_PKTLEN, len);
    RADIOLIB_ASSERT(state);
    // no longer in a direct mode
    cc1101->directModeEnabled = false;
    // update the cached values
    cc1101->packetLength = len;
    cc1101->packetLengthConfig = RADIOLIB_CC1101_LENGTH_CONFIG_INFINITE;

    return state;
}

static TaskHandle_t xHandlingTask = NULL;

#define TX_BIT 0x01
#define RX_BIT 0x02

#define LONG_TIME 0xffff

void cc1101_rx_isr(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Serial.print(F("[CC1101] IRQ!\n"));

  xTaskNotifyFromISR(xHandlingTask, RX_BIT, eSetBits,
                     &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

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
            MIN(cc1101_get_rxfifo_available(cc1101), remaining);
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

struct s_cc1101_rf_rx_settings {
    float freq;
    float br;
    float freqDev;
    float rxBw;
    uint8_t modulation;
};

static BaseType_t cc1101_set_rf(eCC1101 *cc1101, s_cc1101_rf_rx_settings *settings) {
  cc1101->setFrequency(settings->freq);
  cc1101->setBitRate(settings->br);
  cc1101->setFrequencyDeviation(settings->freqDev);
  cc1101->setRxBandwidth(settings->rxBw);
  cc1101->setOOK(settings->modulation == RADIOLIB_CC1101_MOD_FORMAT_ASK_OOK);

  return pdFALSE;
}

static BaseType_t cc1101_receive(eCC1101 *cc1101, s_cc1101_rf_rx_settings *settings, unsigned char *data, size_t len) {
  xHandlingTask = xTaskGetCurrentTaskHandle();

  int state;

  cc1101->setPromiscuousMode(true, false);
  cc1101->setPacketReceivedAction(cc1101_rx_isr);
  state = cc1101->SPIsetRegValue(RADIOLIB_CC1101_REG_PKTCTRL1,
                                 RADIOLIB_CC1101_APPEND_STATUS_OFF, 3, 3);

  state = cc1101_set_rf(cc1101, settings);
  state = cc1101->disableAddressFiltering();
  cc1101->SPIsetRegValue(RADIOLIB_CC1101_REG_MCSM1, RADIOLIB_CC1101_RXOFF_RX, 3,
                         2);
  cc1101_setInfiniteLengthMode(cc1101);

  size_t received = cc1101_receiveStream(cc1101, data, len);
  cc1101->clearPacketReceivedAction();
  cc1101->SPIsendCommand(RADIOLIB_CC1101_CMD_FLUSH_RX);
  cc1101->setPromiscuousMode(false, false);

  return received;
}

static BaseType_t cc1101_scan(eCC1101 *radio, FrequencyRSSI *frequency_rssi) {
  int rssi;
  *frequency_rssi = {.frequency_coarse = 0,
                     .rssi_coarse = -100,
                     .frequency_fine = 0,
                     .rssi_fine = -100};

  // First stage: coarse scan
  radio->setRxBandwidth(
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
      radio->setFrequency((float)frequency / 1000000.0);
      radio->receiveDirect();
      delay(2);
      rssi = radio->getRSSI();

      if (frequency_rssi->rssi_coarse < rssi) {
        frequency_rssi->rssi_coarse = rssi;
        frequency_rssi->frequency_coarse = frequency;
      }
    }
  }

  // Second stage: fine scan
  if (frequency_rssi->rssi_coarse > rssi_threshold) {
    // for example -0.3 ... 433.92 ... +0.3 step 20KHz
    radio->setRxBandwidth(58);
    for (uint32_t i = frequency_rssi->frequency_coarse - 300000;
         i < frequency_rssi->frequency_coarse + 300000; i += 20000) {
      uint32_t frequency = i;
      radio->setFrequency((float)frequency / 1000000.0);
      radio->receiveDirect();
      delay(2);
      rssi = radio->getRSSI();

      if (frequency_rssi->rssi_fine < rssi) {
        frequency_rssi->rssi_fine = rssi;
        frequency_rssi->frequency_fine = frequency;
      }
    }
  }

  return pdFALSE;
}

static BaseType_t cc1101_init_cmd(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString) {
  BaseType_t ret = pdTRUE;
  BaseType_t idLen;
  char buf[16];
  const char *idStr = FreeRTOS_CLIGetParameter(pcCommandString, 1, &idLen);

  int id = atoi(idStr);

  eCC1101 *cc1101 = cc1101_init(id);

  if (cc1101 == NULL)
      return pdFALSE;

  sprintf(pcWriteBuffer, "[CC1101] Module %d initialized!\n", id);
  xWriteBufferLen = strlen(pcWriteBuffer);

  return pdFALSE;
}

FREERTOS_SHELL_CMD_REGISTER("init", "init <radio id>", cc1101_init_cmd, 1);

static BaseType_t cc1101_scan_cmd(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString) {
  int id;
  static int scan_loop = 0;
  size_t len = 0;
  static TickType_t startTime;

  if (pCC1101 == NULL) {
    FreeRTOS_CLIGetParameterAsInt(pcCommandString, 1, &id);
    FreeRTOS_CLIGetParameterAsInt(pcCommandString, 2, &scan_loop);

    pCC1101 = cc1101_init(id);
    if (pCC1101 == NULL)
        return pdFALSE;

    snprintf(pcWriteBuffer, xWriteBufferLen,
             "[CC1101] Module %d initialized!\n "
             "[CC1101] Frequency scanning in progress (%d times)... \n",
             id, scan_loop);
    len = strlen(pcWriteBuffer);
    xWriteBufferLen -= len;
    pcWriteBuffer += len;
    startTime = pdTICKS_TO_MS(xTaskGetTickCount());
  }

  while (scan_loop-- > 0) {
    FrequencyRSSI rssi_scan;
    cc1101_scan(pCC1101, &rssi_scan);
    if (rssi_scan.rssi_fine > rssi_threshold) {
      // Deliver results fine
      len = snprintf(pcWriteBuffer, xWriteBufferLen,
                     "FINE        Frequency: %.2f  RSSI: %d\n",
                     (float)rssi_scan.frequency_fine / 1000000.0,
                     rssi_scan.rssi_fine);
    } else if (rssi_scan.rssi_coarse > rssi_threshold) {
      // Deliver results coarse
      len = snprintf(pcWriteBuffer, xWriteBufferLen,
                     "COARSE      Frequency: %.2f  RSSI: %d\n",
                     (float)rssi_scan.frequency_coarse / 1000000.0,
                     rssi_scan.rssi_coarse);
    }
    if (len)
      return pdTRUE;
  }

  pCC1101 = NULL;
  scan_loop = 0;

  TickType_t endTime = pdTICKS_TO_MS(xTaskGetTickCount());
  snprintf(pcWriteBuffer, xWriteBufferLen,
           "[CC1101] Scanning completed (%u ms)\n", endTime - startTime);

  return pdFALSE;
}
FREERTOS_SHELL_CMD_REGISTER("scan", "scan <radio id> <scan loop>", cc1101_scan_cmd, 2);

static BaseType_t cc1101_receive_cmd(char *pcWriteBuffer,
                                     size_t xWriteBufferLen,
                                     const char *pcCommandString) {
  size_t len = sizeof(dataBuff);
  if (pCC1101 == NULL) { 
    int id;

    FreeRTOS_CLIGetParameterAsInt(pcCommandString, 1, &id);

    pCC1101 = cc1101_init(id);
    if (pCC1101 == NULL)
        return pdFALSE;

    FreeRTOS_CLIGetParameterAsInt(pcCommandString, 2, (int *)&rxLength);
    rxLength = ((rxLength + len - 1) / len) * len;
  }
 
  while ((wrPtr > rxPtr) && (xWriteBufferLen >= 2)) {
    if (rxPtr == dataBuff) {
          snprintf(pcWriteBuffer, xWriteBufferLen, "\n[%02u] ", (unsigned) (rxReceived / len));
          pcWriteBuffer += 6;
          xWriteBufferLen += 6;
      }
    snprintf(pcWriteBuffer, xWriteBufferLen, "%02x", *rxPtr++);
    pcWriteBuffer += 2;
    xWriteBufferLen -= 2;
  }

  if (rxPtr >= (dataBuff + len)) {
    rxPtr = dataBuff;
    wrPtr = dataBuff;
    if (rxReceived >= rxLength) {
      rxReceived = 0;
      pCC1101 = NULL;
      return pdFALSE;
    }
  }

  struct s_cc1101_rf_rx_settings settings433M250kASK = {
    .freq = 433.92,
    .br = 10.0,
    .freqDev = 0.0,
    .rxBw = 250.0,
    .modulation = RADIOLIB_CC1101_MOD_FORMAT_ASK_OOK,
  };

  if ((rxReceived == 0) || (wrPtr == rxPtr)) {
    size_t received = cc1101_receive(pCC1101, &settings433M250kASK, dataBuff, len);
    wrPtr += received;
    rxReceived += received;
  }

  return pdTRUE;
}
FREERTOS_SHELL_CMD_REGISTER("rx", "rx <radio id> <length>", cc1101_receive_cmd, 2);
