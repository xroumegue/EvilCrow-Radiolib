#ifndef __FREERTOS_SHELL_PORT_H
#define __FREERTOS_SHELL_PORT_H

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif
EXTERNC void FreeRTOS_ShellOutput(const char *buffer, int length);

#endif /* __FREERTOS_SHELL_PORT_H */
