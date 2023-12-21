# MotorLogCode
 
## Description of the project

This is the code of a datalogger for a industrial motor (1.5 kW). We used Arduino IDE to upload this code to a ESP32-S3 dev module. This choice of a MCU is important because of the input matrix and the sheer power of the processor. We can assign SPI, I2C and others to almost any port (others can't), more GPIO and all around, better adaptability to future expansions.

## Why pcnt.h over pulse_cnt.h

In the timeframe I developed this code, apparently pulse_cnt hadn't been implemented into [arduino-esp32](https://github.com/espressif/arduino-esp32 "arduino-esp32") framework. So I couldn't use it, but the older implementation worked fine. In the future or if you are getting inspiration from this project, consider using the pulse_cnt.h library, it is referred to in the official docs and it has way more info.

The use of this library is important because it gives us native access to the Pulse Counter modules (PCNT). It has a bunch of Esp32 API commands that enables us to count rises and falls on ports in quite a easy and non-blocking way.

## What is implemented

The system runs freeRTOS, and tasks for each one of the measurement or communication procedures. This implementation has wifi connection, SD and MQTT logging, Speed and Torque measurment, and power calculation. Wifi and MQTT have reconnection procedures, and an offline mode if wifi connect on boot timeouts. SD data is logged with time collected from NTP server. All configuration variables are on top of the program. No queues or mutex implemented, given that the project didn't require it.

Our datalogger has the following modules connected:

* HX711, for load cell readings
* LCD 16x2, serial connected
* TCRT5000, digital pin
* MicroSD

You could change any of this modules. Maybe a Hall effect sensor for speed monitoring, or change the LCD for a better display.

Feel free to ask any questions, my code is available as-is.