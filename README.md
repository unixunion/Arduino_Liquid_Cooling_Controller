# Arduino_Liquid_Cooling_Controller

Monitors 4 Dallas 1-wire temperature sensors and displays them on a SSD1306 OLED. A piezo beeps when temps
exceed thresholds which can be edited through 3 buttons, and settings are saved to EEPROM.

# Current Status

* Monitor 4 Temperature Probes
* Alarms when any probe exceeds a configurable threshold

# Further Plans

* Control of Pumps
* Control of Fans
* Feading tacometer data back to CPU Fan Header / Chassis Fan Header

# Parts

1x SSD1306 128x64 Oled like the Adafruit 0.96" or many of the Generics
4x DS18B20 Temperature probes
3x 10k resistors
3x buttons
1x PC Speaker piezo kind
Arduino UNO
Arduino Screw Shield to make thing easier

