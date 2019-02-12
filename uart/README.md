# UART on STM32
This example shows the usage of UART on STM32 (tested on F411RE with a transciever). It is simple: UART1 must be configured in ST CubeMX ("IDE" by ST, available on their website after registration), then cables must be connected to the board and finally there is an intuitive HAL functionality to write over the transceiver.

The transceiver should be recognized automatically by Linux (tested on Ubuntu 18.04). To read messages,

	make # here
	st-flash write debug/uart.bin 0x8000000
	sudo picocom -b 115200 /dev/ttyUSB0

<img src="layout cubemx.png" alt="layout CubeMX" height="400"/><img src="layout HW.png" alt="layout HW" height="400"/>