// Console IO is a wrapper between the actual in and output and the console code
// In an embedded system, this might interface to a UART driver.

#include "consoleIo.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>

extern UART_HandleTypeDef huart1;


eConsoleError ConsoleIoInit(void)
{
	return CONSOLE_SUCCESS;
}

// This is modified for the Wokwi RPi Pico simulator. It works fine 
// but that's partially because the serial terminal sends all of the 
// characters at a time without losing any of them. What if this function
// wasn't called fast enough?
eConsoleError ConsoleIoReceive(uint8_t *buffer, const uint32_t bufferLength, uint32_t *readLength)
{
	uint32_t i = 0;
	char ch;

	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
  	ch = getchar();
  	putchar(ch); // echo
		buffer[i] = (uint8_t) ch;
		i++;
	}

	*readLength = i;
	return CONSOLE_SUCCESS;
}

eConsoleError ConsoleIoSendString(const char *buffer)
{
	printf("%s", buffer);
	return CONSOLE_SUCCESS;
}

