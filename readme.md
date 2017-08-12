#NMEA2000 Gateway

A NMEA2000 Gateway which receives NMEA0183 GPS messages (like Actisense) and receives compass heading from a HMC5883L sensor.

Based on the NMEA0183ToN2k example from the NMEA0183 library from [Timo Lappalainen](https://github.com/ttlappalainen).
The NMEA0183 library has been extended with additional messages.
This gateway has been succesfully used between a Garmin GPS 120 (NMEA0183) and a B&G Triton display (NMEA2000).
The compass heading is still work in progress and is based on the Arduino-HMC5883L library from Korneliusz Jarzebski.

#Arduino library dependencies

* [NMEA0183](https://github.com/tonswieb/NMEA0183)
	* [Time](https://github.com/PaulStoffregen/Time). A dependency for NMEA0183.
* [NMEA2000](https://github.com/ttlappalainen/NMEA2000)
	* [NMEA2000_mcp](https://github.com/ttlappalainen/NMEA2000_mcp).
	* [CAN\_BUS\_Shield](https://github.com/ttlappalainen/CAN_BUS_Shield). 
* [StandardCPlusPlus](https://github.com/maniacbug/StandardCplusplus)
* [MemoryFree](https://github.com/McNeight/MemoryFree)
* [Arduino-HMC5883L](https://github.com/jarzebski/Arduino-HMC5883L)

The depedencies for NMEA2000 can differ depending on the Arduino board and CAN controller used. See the documentationo of the NMEA2000 library in case of another setup. Above dependencies work for Arduino Mega with MCP2515 CAN bus controller.

#Configuration
The sketches is configured for a MCP2515 CAN bus controller on 8 Mhz by adding the following.
<pre>
#define USE_MCP_CAN_CLOCK_SET 8
</pre>
Remove this line if you have a 16 Mhz CAN bus controller.

The sketch uses `&Serial3` for the NMEA0183 serial interface which is available on Arduino Mega, but not all Arduino boards have more then one serial interface.


