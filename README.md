# TSL2561-DRIVER
Interfacing STM NUCLEO-L073RZ  with TSL2561 Light sensor including Interrupt 

## Hardware & Software
* STM NUCLEO-L073RZ 
* TSL2561 MODULE 
* STM32CubeMx IDE 

## Pin Connection
TSL2561 Sensor interfaced on *I2C* bus over Nucleo Board.  
<pre>
TSL2561 ->  Nucleo Board(Arduino Header)  
VIN     ->  5 Volt
GND     ->  GND
SCL     ->  D15 
SDA     ->  D14
</pre>   
Note: While Doing Connection keep the Solder Bridges of Nucleo Board in mind. 

Threshold Value For Interrupt is purely experimental. Based on the Channel 0's Reading value can be set. 

## Resources 
Application code can be found under *Core/src/main.c*.   
For Adding Serial Printf function , I followed Shawn Hymel's [Tutorial](https://shawnhymel.com/1873/how-to-use-printf-on-stm32/).  
Original Adafruit's Library From which i derived this Project's library can be found [here](https://github.com/adafruit/Adafruit_TSL2561).
