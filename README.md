# SoftRF &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;

DIY, multifunctional, compatible, sub-1 GHz ISM band radio based proximity awareness system for general aviation.
<br>
[User Guide](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/SoftRF_MB_user_guide.txt)
<br>

### Latest additions:

* vMB123: supports baro sensor on the T-Echo ("Badge")
* vMB120: supports the latest 2024 radio protocol
* vMB114: experimental: import traffic data in GDL90 format (from ADS-B receiver)
* vMB110: added second serial port and data bridging (only on T-Beam)
* vMB10h: added winch mode ("aircraft type") with variable reported altitude
* vMB10f: auto turn-off Bluetooth upon web access to resolve memory limitation
* vMB10d: now works on the new v1.2 model of the T-Beam with AXP2101

### Beyond Lyusupov's version:

* Collision prediction for circling aircraft
* Can set aircraft ID for self, ID to ignore, and ID to follow
* Support 3-level collision alarms via buzzer
* Can configure two simultaneous NMEA output destinations
* Settable baud rate for serial output
* Estimates wind while circling, uses for collision prediction
* Corrected frequency hopping and time slots
* Airborne devices relay radio packets from landed aircraft 
* Louder buzzer via 2-pin differential drive, or external
* Collision-danger traffic VOICE warnings! (not just beeps)
* Can adjust SoftRF settings within T-Echo (without an app)
* Includes strobe-control logic
* Option to connect to ambient WiFi network instead of creating one
* Option to send data as TCP client instead of TCP server
* Specify server's IP address for TCP client, and choice of 2 ports
* Ability to use WiFi TCP & UDP NMEA outputs simultaneously
* These WiFi options allow wireless output to XCvario
* Detect and use OLED display on either I2C port
* Modified version of [SkyView](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32/SkyView)
* [SkyStrobe](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32/SkyStrobe) - a controller for a visibility strobe (and more)
* [Documentation files](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/documentation).

Source code, and compiled binaries for [ESP32](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32/SoftRF) and [nRF52](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/nRF52840/SoftRF/MassStorage) (only), are available here.
<br>

For discussions join the [SoftRF Community](https://groups.google.com/g/softrf_community).
<br>
<br>

For additional info see also [Lyusupov's repository](https://github.com/lyusupov/SoftRF).


