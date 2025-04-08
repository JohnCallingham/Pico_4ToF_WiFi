# Pico_4ToF_Wifi
This program provides:
- A Raspberry Pi Pico W connects to the JMRI OpenLCB/LCC hub over WiFi.
- The Pi Pico W connects to a PCA9546A (https://www.adafruit.com/product/5664) I2C 4 way multiplexor via a LTC4311 (https://www.adafruit.com/product/4756) I2C extender.
- Up to 4 VL6180 (https://www.adafruit.com/product/3316) time of flight (ToF) sensors can be connected via I2C to the multiplexor.
- JMRI's OpenLCB/LCC configuraion tool can be used to define 4 distance thresholds per ToF sensor.
- Each distance threshold comprises a near and far distance and near and far events.
- When a sensor detects an object closer than the near distance it sends the near event.
- When a sensor detects an object further away than the far distance it sends the far event.
- These events can be used in JMRI to create a JMRI sensor object for each of the 4 distance thresholds.

Notes:
- The second I2C interface is used to avoid any conflict with the first SPI interface which is used for WiFi.
- The two Adafruit libraries (Adafruit_I2CDevice and Adafruit_VL6180X) are included in the src folder as they are not available in the PlatformIO system.

