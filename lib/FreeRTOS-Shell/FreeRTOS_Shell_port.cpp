#include "FreeRTOS_Shell_port.h"
#include "Arduino.h"
#include "FreeRTOS_Shell.h"

static const uint32_t uartRxTimeout = 1;
static const uint32_t uartBaudrate = 115200;

EXTERNC void FreeRTOS_ShellOutput(const char *buffer, int length) {
  Serial.write(buffer, length);
}

// void serialEvent(void) {
void FreeRTOS_Shell_cb(void) {
  while (Serial.available() > 0) {
    uint8_t incomingByte = Serial.read();
    FreeRTOS_ShellIRQHandle(incomingByte);
  }
}

void FreeRTOS_Shell_init(void) {
  Serial.setRxBufferSize(512);
  Serial.begin(uartBaudrate);
  while (!Serial) {
  };

  Serial.setRxTimeout(uartRxTimeout);
  Serial.onReceive(FreeRTOS_Shell_cb, true);
}
