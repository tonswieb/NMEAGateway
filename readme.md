#NMEA2000 Gateway

A NMEA2000 Gateway which receives NMEA0183 GPS messages (like Actisense) and receives compass heading from a HMC5883L sensor.

Based on the NMEA0183ToN2k example from the NMEA0183 library from [Timo Lappalainen](https://github.com/ttlappalainen).
The NMEA0183 library has been extended with additional messages.
This gateway has been succesfully used between a Garmin GPS 120 (NMEA0183) and a B&G Triton display (NMEA2000).
The compass heading is still work in progress and is based on the Arduino-HMC5883L library from Korneliusz Jarzebski.

#Arduino library dependencies

* [NMEA0183 v2.1.0](https://github.com/tonswieb/NMEA0183/tree/2.1.0)
	* [Time](https://github.com/PaulStoffregen/Time). A dependency for NMEA0183.
* [NMEA2000](https://github.com/ttlappalainen/NMEA2000)
	* [NMEA2000_mcp](https://github.com/ttlappalainen/NMEA2000_mcp)
	* [CAN\_BUS\_Shield](https://github.com/ttlappalainen/CAN_BUS_Shield)
* [MemoryFree](https://github.com/McNeight/MemoryFree)
* [Arduino-HMC5883L](https://github.com/jarzebski/Arduino-HMC5883L)

The depedencies for NMEA2000 can differ depending on the Arduino board and CAN controller used. See the documentationo of the NMEA2000 library in case of another setup. Above dependencies work for Arduino Mega with MCP2515 CAN bus controller.

##Memory usage
This sketch including all Arduino libraries and dependencies require around 6KB of SRAM, so you will need at least Arduino Mega to run it.

#Configuration

##CAN Bus clock speed
The sketches is configured for a MCP2515 CAN bus controller on 8 Mhz by adding the following.
<pre>
#define USE_MCP_CAN_CLOCK_SET 8
</pre>
Remove this line if you have a 16 Mhz CAN bus controller.

## GPS Baudrate
The sketch uses `&Serial3` for the NMEA0183 serial interface which is available on Arduino Mega, but not all Arduino boards have more then one serial interface. Make sure the baudrate set for `&Serial3` matches the baudrate set on the GPS. According to the NMEA 0183 specification it should be 4800, but most GPS support higher baudrates. Normally the higher the better as GPS data is received more frequently. Especially for long routes (RTE, WPL) messages it can speed up sending a GPS route onto the NMEA2000 network. A route cannot be send onto the NMEA2000 network until the RTE messages and all corresponding WPL messages are received. A single WPL message is send once per message cycle followed by a RTE message sequence in a separate cycle. So a route of 30 waypoints requires 31 message NMEA0183 message cycles before the complete route is received by the gateway. See: [NMEA0183 data](www.gpsinformation.org/dale/nmea.htm)

##Defaults
* MAX\_WP\_PER\_ROUTE: The maximum number of waypoints per route. Default = 30. Can be changed by adding:
`#define MAX_WP_PER_ROUTE=30` add the top of the sketch.
* NMEA0183\_MAX\_WP\_NAME\_LENGTH: The maximum length of a waypoint name received via the NMEA0183. Default = 20. Can be changed by adding:
`#define NMEA0183_MAX_WP_NAME_LENGTH=20` add the top of the sketch.
* NMEA0183\_MAX\_WP\_IN\_RTE: The maximum number of waypoints that can be transmitted via a RTE message. Default = 31. Can be changed by adding:
`#define NMEA0183_MAX_WP_IN_RTE=31` add the top of the sketch. Adjust with care. Increasing this default is useless, because a [NMEA 0183 RTE sentence](http://www.gpsinformation.org/dale/nmea.htm#RTE) is limited to 80 characters, but perhaps later specs will get different max. message sizes. Decreasing the default could save some memory, but could lead to waypoints in the RTE message being ignored because the buffer is filled.




