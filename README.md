
 # RPM-Sensor
 
### Release 1.0 / 2025-04-12

 Usage for measuring RMP (Revolutions per Minute) with an infrared sensor 
 and displaying the result on an OLED display. The DS3231 RTC is used to keep track of time.

 The SD card is used to log the data. The program allows setting the RTC time using buttons on the ESP32 board.

 The program is designed to run on an ESP32 board with a DS3231 RTC, an OLED display, and an SD card module, uses the TFT_eSPI library for the OLED display and the RTClib library for the DS3231 RTC.

 The program also uses the SPI library for communication with the SD card module.
 The program is designed to be used with the ESP32 board and is not compatible with other boards.

 ESP32 DS3231 RTC + OLED Display + SD Card + Infrared Sensor
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version.
 
#### Important! 

To avoid problems concerning the SD-card use max. 16 GB SD-card or even better with a capacity of 8 GB. I had a lot of problems with 32 GB cards of various brands!

### RELEASE 2.0 HR 2025-04-22 NK
-------------------------------------------------
Changed from TFT + SD to OLED display
### RELEASE 2.1 HR 2025-04-23 NK
-------------------------------------------------




RPM Sensor User Manual
-----------------------

Overview
The RPM Sensor is a device based on ESP32 that measures, displays, and records the RPM (Revolutions Per Minute) of motors. It features dual OLED displays, real-time clock, SD card logging, and EEPROM backup storage.

Specifications
Microcontroller: ESP32
Displays: 2x OLED displays (128x64 pixels)
Sensors: Hall effect sensor for RPM measurement
Storage: SD card for data logging + EEPROM backup
Real-time Clock: DS3231 for accurate timekeeping
Power Supply: 5V DC

Hardware Connections
--------------------
Pin Configuration:

Component	GPIO Pin	Description
Hall Sensor	GPIO15	RPM detection input (with pull-up) orange/yellow cable

SD Card SPI CS	GPIO5	SD card chip select blue cable
SD Card SPI MOSI	GPIO23 yeollow cable
SD Card SPI MISO	GPIO19	white cable
SD Card SPI CLK      GPIO18 green cable
Button SET	GPIO27	Yellow cable for settings
Button PLUS	GPIO25	Blue cable to increase values
Button MINUS	GPIO26	White cable to decrease values

LED	GPIO12	Status indicator LED

I2C Main Bus SDA	GPIO21	Data line for displays and RTC blue cable
I2C Main Bus SCL	GPIO22	Clock line for displays and RTC green cable
I2C EEPROM Bus SDA	GPIO16	Data line for EEPROM blue cable
I2C EEPROM Bus SCL	GPIO17	Clock line for EEPROM yellow cable

Circuit Diagram
---------------



                                                ┌──────────┐
                                                │          │
                                      3.3V ──┬──┤ DS3231   │
                                             │  │ RTC      │
                              4.7kΩ    4.7kΩ │  │          │
                               ┌─┴─┐    ┌─┴─┐│  │          │
ESP32                          │   │    │   ││  │          │
┌───────────────┐     3.3V ────┴───┴────┴───┴┤  │          │
│               │──────────────────────────┐ │  └──┬───┬───┘
│           SDA │── GPIO21 ────────────────┴─┤     │   │
│           SCL │── GPIO22 ────────────────┬─┤     │   │
│               │                          │ │     │   │
│               │                  ┌───────┴─┤  ┌──┴───┴──┐
│               │                  │  OLED 1  │  │  OLED 2 │
│               │                  │ 0x3C     │  │  0x3D   │
│               │                  └──────────┘  └─────────┘
│               │
│        GPIO15 │───┬─── Hall Sensor Input
│               │   └─── 10kΩ Pull-up to 3.3V
│               │         100nF Capacitor to GND
│               │
│        GPIO27 │───┬─── SET Button ──── GND
│               │   └─── 10kΩ Pull-up to 3.3V
│               │
│        GPIO25 │───┬─── PLUS Button ─── GND
│               │   └─── 10kΩ Pull-up to 3.3V
│               │
│        GPIO26 │───┬─── MINUS Button ── GND
│               │   └─── 10kΩ Pull-up to 3.3V
│               │
│        GPIO5  │─────── SD Card CS
│               │─────── SD Card MOSI
│               │─────── SD Card MISO
│               │─────── SD Card SCK
│               │
│        GPIO16 │───┬─── EEPROM SDA
│               │   └─── 4.7kΩ Pull-up to 3.3V
│               │
│        GPIO17 │───┬─── EEPROM SCL
│               │   └─── 4.7kΩ Pull-up to 3.3V
│               │
│        GPIO12 │───┬─── Status LED
│               │   └─── 220Ω Resistor to GND
└───────────────┘

Signal Conditioning
--------------------
For reliable operation, especially in noisy environments:

Hall Sensor Input (GPIO15):

10kΩ pull-up resistor to 3.3V
100nF ceramic capacitor between GPIO15 and GND for noise filtering
Button Inputs (GPIO25, 26, 27):

10kΩ pull-up resistors to 3.3V
100nF ceramic capacitors between each button pin and GND for debouncing
I2C Lines:

4.7kΩ pull-up resistors on all SDA and SCL lines
For long wires (>10cm), consider adding 100pF capacitors from each line to GND

Operation Guide
---------------
Initial Setup
Connect the ESP32 to a 5V power supply.

The device will initialize:

Both displays will show "Initializing..."
I2C bus test will be performed
SD card and EEPROM will be initialized
After initialization:

Left display: Date and time
Right display: Current RPM value

Setting Date and Time
---------------------
Press the SET button to enter setup mode.

The display will prompt to set the day:

Use PLUS and MINUS buttons to adjust value
Press SET to confirm and move to the next setting
Continue setting month, year, hour, minute, and second in sequence.

After setting the second, the device will save all settings and return to normal mode.

RPM Measurement
---------------
Place the Hall sensor near the rotating part with magnets.
The device requires 4 pulses per revolution (4 magnets recommended).
The RPM will be shown on the right display.
A status LED will blink briefly with each detected pulse.

Data Logging
------------
The device automatically logs RPM data in three ways:

SD Card: Creates CSV files with timestamp, RPM, and temperature

File format: /rpm_log_YYYY-MM-DD_HH-MM-SS.csv
Logging occurs every 2 seconds
EEPROM Backup: Stores data even if SD card is unavailable

Records data every 5 seconds while motor is running
Limited to the EEPROM capacity (512KB)
Motor Stop Detection: Automatically detects when the motor stops

Logs a "Motor stopped" event

Downloading Data:
------------------
Connect the device to a computer via USB and use the serial monitor (115200 baud) to send commands:

download: Download all data from EEPROM to the serial monitor
erase or clear: Erase all data stored in EEPROM
status or info: Show EEPROM usage statistics
Interpreting the Display
Left Display (Time/Date):

First line: Current date (DD.MM.YYYY)
Second line: Current time (HH:MM:SS)
If no SD card, displays "NO CARD"
Right Display (RPM):

Shows current RPM value
Updates in real-time as motor speed changes
Error Handling
SD Card Error: "SD Card Error" will be shown if the card is not detected
RTC Error: "RTC Error" will be shown if the clock module is not detected
If EEPROM is not available, the device will still function, but backup storage will be disabled

Maintenance Tips
----------------
SD Card: Format SD card as FAT32 before first use
Battery: The RTC module contains a coin cell battery (CR2032) which should be replaced every 2-3 years
Hall Sensor: Keep the sensor within 5mm of the magnets for reliable detection
Magnets: Use 4 equally spaced magnets for most accurate readings

Troubleshooting
---------------
Erratic RPM Readings:

Verify proper spacing of magnets (90° apart for 4 magnets)
Check distance between Hall sensor and magnets (should be <5mm)
Ensure Hall sensor is properly connected with pull-up resistor
No SD Card Detection:

Verify card is formatted as FAT32
Check card connections and orientation
Try a different SD card
Display Issues:

Check I2C connections and pull-up resistors
Verify display addresses (0x3C for time, 0x3D for RPM)

Time Keeps Resetting:
Replace the RTC module battery (CR2032)

Technical Specifications
-------------------------
RPM Range: 500-5000 RPM
Sampling Rate: Up to 6000 pulses per second
Data Storage: Up to 10,000 records in EEPROM
SD Card: FAT32 format, up to 32GB
Temperature Range: -10°C to 85°C
Measurement Accuracy: ±1% after calibration

