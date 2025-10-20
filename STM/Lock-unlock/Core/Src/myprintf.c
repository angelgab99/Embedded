#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "myprintf.h"
UART_HandleTypeDef* gHuart;

void RetargetInit(UART_HandleTypeDef *huart) {
  gHuart = huart;
  /* Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed. */
  setvbuf(stdout, NULL, _IONBF, 0);
}
int _write(int fd, char* ptr, int len) {
  HAL_UART_Transmit(gHuart, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  return len;
}
