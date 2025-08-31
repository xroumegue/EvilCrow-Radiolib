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

#define ALIGN(x, y) (((x + y - 1) / y) * y) 
#define MAX(x, y) (x < y ? y : x)
#define MIN(x, y) (x < y ? x : y)

static eCC1101 ecrf_radios[] = {
    eCC1101(cc1101_mod1_pins, spi, cs_unused_mod1, SPI_CLK_FREQ),
    eCC1101(cc1101_mod2_pins, spi, cs_unused_mod2, SPI_CLK_FREQ),
};

static ssize_t rxReceived = 0;
static ssize_t rxLength;
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


#define LONG_TIME 0xffff

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
  const size_t minLength = 32;
  const size_t maxLineLength = 128;
  if (pCC1101 == NULL) { 
    int id;

    FreeRTOS_CLIGetParameterAsInt(pcCommandString, 1, &id);

    pCC1101 = cc1101_init(id);
    if (pCC1101 == NULL)
        return pdFALSE;

    FreeRTOS_CLIGetParameterAsInt(pcCommandString, 2, (int *)&rxLength);
    rxLength = ALIGN(rxLength, minLength);

    struct s_cc1101_rf_rx_settings settings433M250kASK = {
      .freq = 433.92,
      .br = 10.0,
      .freqDev = 0.0,
      .rxBw = 250.0,
      .modulation = RADIOLIB_CC1101_MOD_FORMAT_ASK_OOK,
    };
    pCC1101->startRawReceive(&settings433M250kASK);
  }
 
  if ((rxReceived % maxLineLength) == 0 ) {
    snprintf(pcWriteBuffer, xWriteBufferLen, "\n[%02u] ", (unsigned) (rxReceived / maxLineLength));
    pcWriteBuffer += 6;
    xWriteBufferLen += 6;
  }

  char *rxPtr = pcWriteBuffer + xWriteBufferLen / 2;
  //size_t xferLen = ALIGN(MIN(xWriteBufferLen / 2, rxLength), minLength);
  size_t xferLen = MIN(xWriteBufferLen / 2, rxLength);

  int len = pCC1101->rawReceive((uint8_t *)rxPtr, xferLen, pdMS_TO_TICKS(5000));
  //char buf[64];
  //snprintf(buf, sizeof(buf), "[%d]\n", len);
  //Serial.print(buf);

  if (len < 0) {
    Serial.println("Error");
  }

  rxReceived += len;
  while (len--) {
    snprintf(pcWriteBuffer, xWriteBufferLen, "%02x", *rxPtr++);
    pcWriteBuffer += 2;
    xWriteBufferLen -= 2;
  }

  if ((rxLength - rxReceived) > 0) {
    return pdTRUE;
  } else {
    pCC1101->stopRawReceive();
    pCC1101 = NULL;
    rxReceived = 0;
    return pdFALSE;
  }
}
FREERTOS_SHELL_CMD_REGISTER("rx", "rx <radio id> <length>", cc1101_receive_cmd, 2);
