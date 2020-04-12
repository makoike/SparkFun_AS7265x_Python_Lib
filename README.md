SparkFun Triad Spectroscopy Sensor - AS7265x (Qwiic)
====================================================

[*SparkFun Triad Spectroscopy Sensor (SEN-15050)*](https://www.sparkfun.com/products/15050)

[*SparkFun AS7265x Arduino Library*](https://github.com/sparkfun/SparkFun_AS7265x_Arduino_Library)

This library is a Python implementation of the *SparkFun Arduino Library*.

Usage (on RasPi)
=====

```Python
from smbus import SMBus
from as7265x import AS7265X

i2c = SMBus(1)
sensor = AS7265X(i2c)

sensor.begin()

sensor.takeMeasurementsWithBulb()
print("A:{}".format(sensor.getCaibratedA())
print("B:{}".format(sensor.getCaibratedB())
print("C:{}".format(sensor.getCaibratedC())
print("D:{}".format(sensor.getCaibratedD())
```

