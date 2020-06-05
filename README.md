# AgriDaughter

Smart controller for Agriculture that measures Temp, Humidity, Moisture Level and decides when it is required to take action.

## Current Mode of Operation:

Currently the MCu will start, Read Temp value Read Moisture & Humidity values, Output a warning if Moisture is below certain level, and goes to sleep after 10 cycles (each cycle ~ 3 - 4 seconds ). MCU wakes up only after we hit reset button.

## We have the following:

- Arduino Uno Board
- TMP36-1 Sensor Analog
- DHT11 Humidity and Moisture Sensor
- LCD attached
- LowPower implementation
- Solar power chargin abilities (XXX: TODO), 