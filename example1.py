import as7265x
import smbus
import numpy as np
import matplotlib.pyplot as plt

i2c = smbus.SMBus(1)
sensor = as7265x.AS7265X(i2c)

sensor.begin()
sensor.enableBulb(as7265x.LED_WHITE)
sensor.enableBulb(as7265x.LED_IR)
sensor.enableBulb(as7265x.LED_UV)
sensor.setIntegrationCycles(1)

x = ['410', '435', '460', '485', '510', '535', '560', '585',
     '610', '645', '680', '705', '730', '760', '810', '860',
     '900', '940']

while (1):
    try:
        sensor.takeMeasurements()

        data = []
        data.append(sensor.getCalibratedA())
        data.append(sensor.getCalibratedB())
        data.append(sensor.getCalibratedC())
        data.append(sensor.getCalibratedD())
        data.append(sensor.getCalibratedE())
        data.append(sensor.getCalibratedF())
        data.append(sensor.getCalibratedG())
        data.append(sensor.getCalibratedH())
        data.append(sensor.getCalibratedI())
        data.append(sensor.getCalibratedJ())
        data.append(sensor.getCalibratedK())
        data.append(sensor.getCalibratedL())
        data.append(sensor.getCalibratedR())
        data.append(sensor.getCalibratedS())
        data.append(sensor.getCalibratedT())
        data.append(sensor.getCalibratedU())
        data.append(sensor.getCalibratedV())
        data.append(sensor.getCalibratedW())

        plt.plot(x, data)
        plt.ylim(0, 200)
        plt.pause(0.03)
        plt.cla()
    except:
        break

sensor.disableBulb(as7265x.LED_WHITE)
sensor.disableBulb(as7265x.LED_IR)
sensor.disableBulb(as7265x.LED_UV)

