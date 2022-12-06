# Release notes

## revision 01a

#### ESP32

Very first deployment of SkyView's firmware binaries.

by Moshe Braner, Dec. 6, 2022

WHAT IS SKYSTROBE

Gliders (and other aircraft) can benefit from higher visibility to other aircraft.  That can be achieved using bright LEDs mounted under the canopy or elsewhere.  Due to the limited electrical power available, such strobes must be directional (usually facing forward), and turned on only a small percentage of the time.  SkyStrobe is firmware for a computerized control circuit for such an LED strobe.  It controls the flashing pattern.  It can increase the flashing frequency when information about nearby traffic, received via radio signals, warrants it.  It can also keep the strobe from flashing when the glider is on the ground.  A SkyStrobe device can work standalone, or can receive information via a cable or wirelessly.

In addition, since the Skystrobe device runs on the glider's 12V battery system, it can generate louder collision-warning beeps than a SoftRF device running on lower voltage.  The SkyStrobe firmware includes the capability to drive such added hardware, based on the signals it receives (via cable or wirelessly) from a SoftRF device, or a FLARM device.

Skystrobe can also serve as a data bridge.  Data arriving via a wireless connection is mirrored by SkyStrobe to its serial port.  This can be useful to feed data, sent wirelessly from SoftRF, to a device that can process FLARM type input but only via a serial cable, such as a ClearNav, SN10, or XCvario.  Conversely, Skystrobe can mirror data arriving via serial cable (e.g., from FLARM) to WiFi or Bluetooth (e.g., to XCsoar).


ALTERNATIVES TO SKYSTROBE

My current version of SoftRF has strobe-control capability built-in.  One can connect the appropriate pins in the hardware, via wires, to a simple (non-computerized) strobe circuit.  See example circuit diagram here: ___________  But this approach cannot be used when the data source is a FLARM device rather than SoftRF, or when a wireless connection is preferred.


SKYSTROBE HARDWARE

The SkyStrobe software is available compiled for the ESP32 platform.  It only requires a simple ESP32 development board, without the radio devices (beyond the WiFi/Bluetooth capability of the ESP32 module itself).  There are many such boards available inexpensively.  Just make sure it includes 4+ MB flash memory.

For the strobe part, an LED module is needed that can be driven in short pulses with a momentary power on the order of 10 watts (or more).  Red color is preferred.


SETTINGS

SkyStrobe has some settings that can be changed via a web browser in a similar way to SoftRF.  Here are the settings available in the current version:

* Input type: whether via serial cable, WiFi or one of two types of Bluetooth.  If you are using SkyStrobe standalone, unconnected, choose "serial", which is also the default.

* Bridge output: wireless inputs always get mirrored to the serial output (as well as USB).  Serial input can be mirrored to a wireless output if called for in this setting.  Bridging between WiFi and Bluetooth is not possible, due to their using the same hardware in the ESP32.

* Protocol: the data coming in from the connected device is usually in the NMEA format (default), but GDL90 is also theoretically supported (I have not tested it).

* Baud rate: this is only relevant to the serial connection method.  38400 is the default, as it is on SoftRF.

* Source Id: this is the network name (SSID), and password, if connecting via WiFi.

* Strobe: the mode of operation can be "off", "alarm" (only flashes when the connected device is signaling a collision danger), "airborne" (flashes periodically once airborne, and more often when the connected device is signaling a collision danger - the default, and recommended), and "always" (even when not airborne).

* Sound: the beeps upon collision warnings, can turn on or off.  Default is on - but a sound device may not be connected.


OPERATION

SkyStrobe keeps trying its WiFi connection until the other device is ready.

Note that regardless of the strobe operation mode selected (other than "off") the strobe will operate for 2 minutes after SkyStrobe power-on, alternating the "alarm" and non-alarm flashing patterns, and occasionally beeping, as a self-test.

After the self-test, if in "airborne" mode, SkyStrobe will try and detect whether the aircraft is airborne (based on GNSS sentences in the NMEA stream).  If a GNSS fix (from the connected device) is not available, SkyStrobe will assume the aircraft is airborne and will flash the strobe periodically (but not in the faster "alarm" pattern since the data is not available).  This is a "fail-safe" feature.

Alarm beeps will sound whenever another aircraft is first considered to pose a collision danger at one of the three "alarm levels" defined in the FLARM protocol: collision within 8 seconds (level 3), 9-12 seconds (level 2), or 13-19 seconds (level 1).  The warning beep will actually sound a second or two later than the time of that calculation.  After that, the same aircraft will not be warned about again, unless it either reaches a higher alarm level than was already warned, or the alarm level decreases first and then increases again.  The beeping pattern identifies the alarm level: a single beep for level 1, double beep for level 2, and 5 short beeps for level 3.  The pitch of the beep also increases with the level.


MORE DETAILS

See the document file, [here](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/documentation).

