# Electeam-Sharky-ESP32
This is the firmware for the ESP32 WROOM 32 microcontroller where throttle, break, LCD screen, motor and more systems are integrated.
![Schematic](https://drive.google.com/uc?export=view&id=18NEBAM3f7z6DvfUNkW7gQg9mj4XFdRmA) 

This is a result of a modification of the MCPWM BLDC motor control(hall sensor feedback) Example from ESP-IDF library.

Here you have the documentation of the example:

## MCPWM BLDC motor control(hall sensor feedback) Example

This example will show you how to use MCPWM module to control bldc motor with hall sensor feedback
 
The following examples uses MCPWM module to control bldc motor and vary its speed continuously

The bldc motor used for testing this code had hall sensor capture sequence of 6-->4-->5-->1-->3-->2-->6-->4--> and so on

IR2136 3-ph bridge driver is used for testing this example code

User needs to make changes according to the motor and gate driver ic used

 
## Step 1: Pin assignment
* The gpio init function initializes:
	* GPIO19 is assigned as the MCPWM signal for 1H(UH)
	* GPIO18 is assigned as the MCPWM signal for 1L(UL)
	* GPIO17 is assigned as the MCPWM signal for 2H(VH)
	* GPIO16 is assigned as the MCPWM signal for 2L(VL)
	* GPIO15 is assigned as the MCPWM signal for 3H(WH)
	* GPIO14 is assigned as the MCPWM signal for 3L(WL)
	* GPIO23 is assigned as the MCPWM capture signal for Hall A
	* GPIO25 is assigned as the MCPWM capture signal for HALL B
	* GPIO26 is assigned as the MCPWM capture signal for HALL C




## Step 3: Initialize MCPWM
 * You need to set the frequency and duty cycle of MCPWM timer
 * You need to set the MCPWM channel you want to use, and bind the channel with one of the timers
 * You need to set the capture unit, for POS/NEG edge capture
 * Also reversing the hall sensor BIT weight, will make bldc motor rotate CW or CCW
