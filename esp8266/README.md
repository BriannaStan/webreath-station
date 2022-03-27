# webreath-station - esp8266 (base version)

This station is made out of an ESP8266 MCU, BME280 sensor, CCS811 sensor, Sensirion SPS-30 sensor and an I2C LCD 20x4.

For this code I used sensirion-sps, adafruit bme280, adafruit ccs811 and LiquidCrystal_I2C libraries that are available under Arduino IDE manage libraries section.
The rest of includes are ESP8266 based libraries.

The code registers itself to we-breathe.org, saves internally in eeprom it's id and starts sending data every 30s.
