# ot-sleepy-sensor
Sensor node as OpenThread sleepy end device.

Experimental sketch using an Adafruit ItsyBitsy nRF52840 Express board and an alternative [Arduino package by soburi with OpenThread support](https://github.com/soburi/openthread_nrf52_arduino).
To get a complete picture, have a look at [Brezensalzer/ot-nRF52-experiments](https://github.com/Brezensalzer/ot-nRF52-experiments).

## Indirect current measurement Thread module with i2c sensor
- [Adafruit ItsyBitsy nRF52840 Express](https://learn.adafruit.com/adafruit-itsybitsy-nrf52840-express) development board
- [MS8607](https://learn.adafruit.com/adafruit-te-ms8607-pht-sensor/overview) temperature, humidity and pressure sensor via i2c
- Measurement interval: 30s
- Power supply: LiFePO4 battery in AA-Format with 550 mAh, fully charged

**Start time: 07.02.2022, 12:18**<br/>
**measured battery level: 72%**

**End time: 19.02.2022, 12:30 Uhr**
**measured battery level: 10%**

**Runtime: 288h (12 days)**
**Estimated current consumption: 1,7 mA**
(500 mAh / 288h = 1,7 mA)
