This examples shows a theremin in STM32 Nucleo F411RE, using a buzzer and a 5V ultrasonic sensor.

In particular, this example shows:

	* How to use a free-running timer (TIM1)
	* How to use Timers in PWM (Pulse width modulation) mode (TIM3) to control the intensity of a buzzer;
	* How to delay for some microseconds
	* The use of transcievers for debugging

To do that:

	* The ultrasonic sensor Trigger pin raises its intensity for 10 seconds
	* Its output is read from the Echo pin
	* Output duration and object distance are computed
	* The buzzer is given some voltage

On STM32 Nucleo F411RE, connect the buzzer to GND and to D12; connect a transciever to D12 and D2; connect the ultrasonic sensor Trigger pin to D11 and the Echo pin to D5, GND to GND and Voltage to the voltage.
The attached figure shows it too.

<img src="layout HW.jpg" height="400" alt="layout HW"/>

<a href="video theremin.mp4" alt="Video theremin">Video of the theremin project</a>