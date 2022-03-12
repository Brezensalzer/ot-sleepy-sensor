# ot-sleepy-sensor
Sensor node as OpenThread sleepy end device.

Experimental sketch using an Adafruit ItsyBitsy nRF52840 Express board and an alternative [Arduino package by soburi with OpenThread support](https://github.com/soburi/openthread_nrf52_arduino).
To get a complete picture, have a look at [Brezensalzer/ot-nRF52-experiments](https://github.com/Brezensalzer/ot-nRF52-experiments).

The ItsyBitsy was modified to achieve the lowest possible quiescence current:
* the Dotstar LED was removed
* the LDO regulator was removed

The Arduino libraries are not able to deactivate the nordic softmodem once enabled, leading to a quiescence current of 275 µA.
Therefore the sketch starts with the delay() (equivalent to sleep), starts the measurement, connects to openthread and sends the message.
Afterwards a soft reset is performed.

This results in a quiescence current of 2.7 µA.
