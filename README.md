# Dmx_ESP32
A DMX512 library for use with ESP32 boards using the Arduino IDE

Dmx breakdetection is done through the use of the RMT peripheral and reception on the UART.

Compiles and works on Core 2.0.x and from Core 3.3.x onwards. 
Core version 3 had a fatal bug in the RMT driver before 3.3.0 . 
The Uart Rx pin and The RMT Rx pin can be shared on core 2 without issue and on core 3 thanx to
@SuGider . All ESP32 boards have at least 1 RMT Rx channel and at least 1 UART. If the board specification
states the number of RMT channels, Half of them can be used for transmission and half can be used for reception.

Most RS485 transceivers are 5v devices and should be powered with 5v. If you connect a 5v logic device output 
to the ESP32, you should use a voltage divider to reduce the logic level to 3.3v . 3.3v Ouputs can easily drive 
5v logic inputs. DMX specification uses the GND of the master device as a shield but leaves all GND of the other 
devices unconnected to each other. The last device on the chain should have a 120 Ohm resistor between A & B as 
a terminator. This ensures that current flows between the lines and this current will, in combination with the 
twisted pair cable, create a magnetic field that counteracts the capacitance of the cable, which will result in
significantly less degradation of the data signal.

RDM is not supported in this library. Systems that use RDM for their configuration are rare and usually are 
configured using a separate device. The ESP32 has both WiFi and Bluetooth available which can be used as
a user interface for configuration of the device. There are plenty of RDM supporting libraries.
