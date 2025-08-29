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

static eCC1101 *cc1101_init(int id) {

  if ((id < 0) || (id > 1)) {
    Serial.print(F("[E] [CC1101] Wrong module id ... "));
    return NULL;
  }

  eCC1101 *cc1101 = &ecrf_radios[id];
  cc1101->begin();
  cc1101->get_radio_state();
  return cc1101;
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

static BaseType_t cc1101_receive(eCC1101 *cc1101, s_cc1101_rf_rx_settings *settings, unsigned char *data, size_t len) {
  xHandlingTask = xTaskGetCurrentTaskHandle();

  int state;

  cc1101->setPromiscuousMode(true, false);
  cc1101->setPacketReceivedAction(cc1101_rx_isr);
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
  const int rssi_threshold = -75;
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
    pCC1101->scan(&rssi_scan);
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
