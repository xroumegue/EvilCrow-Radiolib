/* FreeRTOS includes. */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/* Standard includes. */
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* custom includes. */
#include "FreeRTOS_CLI.h"
#include "FreeRTOS_Shell.h"
#include "FreeRTOS_Shell_port.h"

/* Private variables ---------------------------------------------------------*/
QueueHandle_t FreeRTOS_ShellRecvQueue;
static uint8_t inputBuffer[FREERTOS_SHELL_INPUT_BUFFER_LENGTH];
static uint8_t *inputBuffer_ptr;
static uint8_t outputBuffer[FREERTOS_CLI_OUTPUT_MAX_BUFFER_SIZE];

/* Extern variables ---------------------------------------------------------*/
extern uint8_t __freertos_shell_cmd_start;
extern uint8_t __freertos_shell_cmd_end;

/* Private function prototypes -----------------------------------------------*/
__attribute__((weak)) void FreeRTOS_Shell_init(void) {}

/**
 * @brief A FreeRTOS thread, it will handle msg from a msgqueue, and output to
 * UART
 *
 * @note  when there is no input, the thread will suspend and take no CPU time.
 */
void FreeRTOS_Shell(void *params) {
  FreeRTOS_Shell_init();
  /* a shell task */
  inputBuffer_ptr = inputBuffer;
  FreeRTOS_ShellRecvQueue =
      xQueueCreate(FREERTOS_SHELL_RECV_QUEUE_LENGTH, sizeof(uint8_t));
  configASSERT(FreeRTOS_ShellRecvQueue);

  FreeRTOS_ShellOutput(FREERTOS_SHELL_START_LOGO,
                       strlen(FREERTOS_SHELL_START_LOGO));
  FreeRTOS_ShellOutput("\r\n", 2);
  FreeRTOS_ShellOutput(FREERTOS_SHELL_USER_INFO,
                       strlen(FREERTOS_SHELL_USER_INFO));

  /* regist all cmd using link symbol */
  uint8_t *start = &__freertos_shell_cmd_start;
  uint8_t *end = &__freertos_shell_cmd_end;
  int size = sizeof(CLI_Command_Definition_t);
  for (uint8_t *i = start; i < end; i += size) {
    CLI_Command_Definition_t *cli_command = (CLI_Command_Definition_t *)i;
    FreeRTOS_CLIRegisterCommand(cli_command);
  }

  while (1) {
    /* always wait a queue */
    char recvChar = 0;
    xQueueReceive(FreeRTOS_ShellRecvQueue, &recvChar, portMAX_DELAY);

    bool lineOver = false;
    bool isInputBufferEmpty = inputBuffer_ptr == inputBuffer;
    if (recvChar == '\r')
      lineOver = true;

    if (lineOver) {
      BaseType_t ret = pdTRUE;
      FreeRTOS_ShellOutput("\r\n", 2);
      if (!isInputBufferEmpty) {
        while (ret == pdTRUE) {
          ret = FreeRTOS_CLIProcessCommand((const char *)inputBuffer,
                                           (char *)outputBuffer,
                                           FREERTOS_CLI_OUTPUT_MAX_BUFFER_SIZE);
          FreeRTOS_ShellOutput((const char *)outputBuffer,
                               strlen((const char *)outputBuffer));
        }
        memset(outputBuffer, 0, FREERTOS_CLI_OUTPUT_MAX_BUFFER_SIZE);
        memset(inputBuffer, 0, FREERTOS_SHELL_INPUT_BUFFER_LENGTH);
        inputBuffer_ptr = inputBuffer;
        FreeRTOS_ShellOutput("\r\n", 2);
      }
      FreeRTOS_ShellOutput(FREERTOS_SHELL_USER_INFO,
                           strlen(FREERTOS_SHELL_USER_INFO));
    } else if (!lineOver) {
      /* backspace */
      if (((recvChar == '\n') || (recvChar == 0x7f)) && isInputBufferEmpty) {
        __asm__("nop");
      }
      /* backspace*/
      else if (recvChar == 0x7f && !isInputBufferEmpty) {
        *--inputBuffer_ptr = 0;
        FreeRTOS_ShellOutput(&recvChar, 1);
      }
      /* tab */
      /* TODO: when double click tab, trigger auto complete */
      else if (recvChar == 0x08) {
        __asm__("nop");
      } else {
        *inputBuffer_ptr++ = recvChar;
        FreeRTOS_ShellOutput(&recvChar, 1);
      }
    }
  }
}

void FreeRTOS_ShellIRQHandle(uint8_t recvData) {
  /* send data to FreeRTOS queue */
  xQueueSendToBackFromISR(FreeRTOS_ShellRecvQueue, &recvData, NULL);
}

static int listAllThreadCallCount = 0;
static BaseType_t listAllThread(char *pcWriteBuffer, size_t xWriteBufferLen,
                                const char *pcCommandString) {
  BaseType_t ret = pdTRUE;
  UBaseType_t taskNum = uxTaskGetNumberOfTasks();
  int len = taskNum * FREERTOS_SHELL_EACH_TASKINFO_MAX_SIZE;

  char *_buffer = (char *)malloc(len * sizeof(char));
  vTaskList(_buffer);
  if (len > xWriteBufferLen * (listAllThreadCallCount + 1)) {
    /* Insufficient buffer size, should be called more than once */
    memcpy(pcWriteBuffer, _buffer + xWriteBufferLen * listAllThreadCallCount,
           xWriteBufferLen);
    listAllThreadCallCount++;
    ret = pdTRUE;
  } else {
    /* sufficient buffer size */
    memcpy(pcWriteBuffer, _buffer + xWriteBufferLen * listAllThreadCallCount,
           xWriteBufferLen * (listAllThreadCallCount + 1) - len);
    listAllThreadCallCount--;
    ret = pdFALSE;
  }

end:
  free(_buffer);
  return ret;
}

FREERTOS_SHELL_CMD_REGISTER("ps", "list all thread", listAllThread, 0);
