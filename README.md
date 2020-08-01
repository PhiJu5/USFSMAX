<a href="https://www.tindie.com/products/onehorse/max32660-motion-co-processor/"><img src="extras/media/usfsmax.jpg" width=500></a>

This repository derives from Greg Tomasch's [code](https://github.com/gregtomasch/USFSMAX) for the Pesky Products
[USFSMAX motion coprocessor](https://www.tindie.com/products/onehorse/max32660-motion-co-processor/).  
See Greg's library for details.

Changes I made to Greg's version:

* Made a proper Arduino repository

* Eliminated globals and #defines

* Moved all printout from library code to sketch

* Simplified arrays; e.g., ```gyroData[2][3]``` => ```gyroData[3]```

* Use [CrossPlatformDataBus](https://github.com/simondlevy/CrossPlatformDataBus) to support multiple host types
(Arduino, RaspberryPi, NVIDIA Jetson); work in progress

I have tested this library on the following hosts:

* TinyPICO ESP32

* Teensy4.0
