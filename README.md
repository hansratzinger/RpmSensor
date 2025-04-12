
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
 

-------------------------------------
### RELEASE 1.1 HR 2025-04-12 NK

Added failure procedure for SD card
Added temperature measurement and logging



