import time
import struct

I2C_ADDR = 0x49

STATUS_REG = 0x00
WRITE_REG = 0x01
READ_REG = 0x02
TX_VALID = 0x02
RX_VALID = 0x01

#Register addresses
HW_VERSION_HIGH = 0x00
HW_VERSION_LOW = 0x01
FW_VERSION_HIGH = 0x02
FW_VERSION_LOW = 0x03

CONFIG = 0x04
INTEGRATION_TIME = 0x05
DEVICE_TEMP = 0x06
LED_CONFIG = 0x07

#Raw channel registers
R_G_A = 0x08
S_H_B = 0x0a
T_I_C = 0x0c
U_J_D = 0x0e
V_K_E = 0x10
W_L_F = 0x12

#Calibrated channel registers
R_G_A_CAL = 0x14
S_H_B_CAL = 0x18
T_I_C_CAL = 0x1c
U_J_D_CAL = 0x20
V_K_E_CAL = 0x24
W_L_F_CAL = 0x28

DEV_SELECT_CONTROL = 0x4F

COEF_DATA_0 = 0x50
COEF_DATA_1 = 0x51
COEF_DATA_2 = 0x52
COEF_DATA_3 = 0x52
COEF_DATA_READ = 0x54
COEF_DATA_WRITE = 0x55

#Settings
POLLING_DELAY = 0.01
NIR = 0x00
VISIBLE = 0x01
UV = 0x02

LED_WHITE = 0x00
LED_IR = 0x01
LED_UV = 0x02
LED_CURRENT_LIMIT_12_5MA = 0b00
LED_CURRENT_LIMIT_25MA   = 0b01
LED_CURRENT_LIMIT_50MA   = 0b10
LED_CURRENT_LIMIT_100MA  = 0b11

INDICATOR_CURRENT_LIMIT_1MA = 0b00
INDICATOR_CURRENT_LIMIT_2MA = 0b01
INDICATOR_CURRENT_LIMIT_4MA = 0b10
INDICATOR_CURRENT_LIMIT_8MA = 0b11

GAIN_1X = 0b00
GAIN_37X = 0b01
GAIN_16X = 0b10
GAIN_64X = 0b11

MEASUREMENT_MODE_4CHAN = 0b00
MEASUREMENT_MODE_4CHAN_2 = 0b01
MEASUREMENT_MODE_6CHAN_CONTINUOUS = 0b10
MEASUREMENT_MODE_6CHAN_ONE_SHOT = 0b11


class AS7265X():
    def __init__(self, i2c_bus):
        self._bus = i2c_bus

    def begin(self):
        if not self.isConnected():
            return False

        value = self.virtualReadRegister(DEV_SELECT_CONTROL)
        if (value & 0b00110000) == 0:
            return False

        self.setBulbCurrent(LED_CURRENT_LIMIT_12_5MA, LED_WHITE)
        self.setBulbCurrent(LED_CURRENT_LIMIT_12_5MA, LED_IR)
        self.setBulbCurrent(LED_CURRENT_LIMIT_12_5MA, LED_UV)

        self.disableBulb(LED_WHITE)
        self.disableBulb(LED_IR)
        self.disableBulb(LED_UV)

        self.setIndicatorCurrent(INDICATOR_CURRENT_LIMIT_8MA)
        self.enableIndicator()

        self.setIntegrationCycles(49) #50 * 2.8ms = 140ms.
        self.setGain(GAIN_64X)
        self.setMeasurementMode(MEASUREMENT_MODE_6CHAN_ONE_SHOT)
        self.enableInterrupt()
        return True

    def getDeviceType(self):
        return self.virtualReadRegister(HW_VERSION_HIGH)

    def getHardwareVersion(self):
        return self.virtualReadRegister(HW_VERSION_LOW)

    def getMajorFirmwareVersion(self):
        self.virtualWriteRegister(FW_VERSION_HIGH, 0x01)
        self.virtualWriteRegister(FW_VERSION_LOW, 0x01)

        return self.virtualReadRegister(FW_VERSION_LOW)

    def getPatchFirmwareVersion(self):
        self.virtualWriteRegister(FW_VERSION_HIGH, 0x02)
        self.virtualWriteRegister(FW_VERSION_LOW, 0x02)

        return self.virtualReadRegister(FW_VERSION_LOW)

    def getBuildFirmwareVersion(self):
        self.virtualWriteRegister(FW_VERSION_HIGH, 0x03)
        self.virtualWriteRegister(FW_VERSION_LOW, 0x03)

        return self.virtualReadRegister(FW_VERSION_LOW)

    def isConnected(self):
        try:
            self._bus.read_byte_data(I2C_ADDR, STATUS_REG)
            return True
        except:
            return False

    def takeMeasurements(self):
        self.setMeasurementMode(MEASUREMENT_MODE_6CHAN_ONE_SHOT)
        
        while not self.dataAvailable():
            time.sleep(POLLING_DELAY)

    def takeMeasurementsWithBulb(self):
        self.enableBulb(LED_WHITE)
        self.enableBulb(LED_IR)
        self.enableBulb(LED_UV)

        self.takeMeasurements()

        self.disableBulb(LED_WHITE)
        self.disableBulb(LED_IR)
        self.disableBulb(LED_UV)

    #Get the various color readings
    def getG(self):
        return self.getChannel(R_G_A, VISIBLE)

    def getH(self):
        return self.getChannel(S_H_B, VISIBLE)

    def getI(self):
        return self.getChannel(T_I_C, VISIBLE)

    def getJ(self):
        return self.getChannel(U_J_D, VISIBLE)

    def getK(self):
        return self.getChannel(V_K_E, VISIBLE)

    def getL(self):
        return self.getChannel(W_L_F, VISIBLE)

    #Get the various NIR readings
    def getR(self):
        return self.getChannel(R_G_A, NIR)

    def getS(self):
        return self.getChannel(S_H_B, NIR)

    def getT(self):
        return self.getChannel(T_I_C, NIR)

    def getU(self):
        return self.getChannel(U_J_D, NIR)

    def getV(self):
        return self.getChannel(V_K_E, NIR)

    def getW(self):
        return self.getChannel(W_L_F, NIR)

    #Get the various UV readings
    def getA(self):
        return self.getChannel(R_G_A, UV)

    def getB(self):
        return self.getChannel(S_H_B, UV)

    def getC(self):
        return self.getChannel(T_I_C, UV)

    def getD(self):
        return self.getChannel(U_J_D, UV)

    def getE(self):
        return self.getChannel(V_K_E, UV)

    def getF(self):
        return self.getChannel(W_L_F, UV)

    def getChannel(self, channelRegister, device):
        self.selectDevice(device)
        colorData = self.virtualReadRegister(channelRegister) << 8
        colorData |= self.virtualReadRegister(channelRegister + 1)
        return colorData

    #Returns the various calibration data
    def getCalibratedA(self):
        return self.getCalibratedValue(R_G_A_CAL, UV)

    def getCalibratedB(self):
        return self.getCalibratedValue(S_H_B_CAL, UV)

    def getCalibratedC(self):
        return self.getCalibratedValue(T_I_C_CAL, UV)

    def getCalibratedD(self):
        return self.getCalibratedValue(U_J_D_CAL, UV)

    def getCalibratedE(self):
        return self.getCalibratedValue(V_K_E_CAL, UV)

    def getCalibratedF(self):
        return self.getCalibratedValue(W_L_F_CAL, UV)

    #Returns the various calibration data
    def getCalibratedG(self):
        return self.getCalibratedValue(R_G_A_CAL, VISIBLE)

    def getCalibratedH(self):
        return self.getCalibratedValue(S_H_B_CAL, VISIBLE)

    def getCalibratedI(self):
        return self.getCalibratedValue(T_I_C_CAL, VISIBLE)

    def getCalibratedJ(self):
        return self.getCalibratedValue(U_J_D_CAL, VISIBLE)

    def getCalibratedK(self):
        return self.getCalibratedValue(V_K_E_CAL, VISIBLE)

    def getCalibratedL(self):
        return self.getCalibratedValue(W_L_F_CAL, VISIBLE)

    #Returns the various calibration data
    def getCalibratedR(self):
        return self.getCalibratedValue(R_G_A_CAL, NIR)

    def getCalibratedS(self):
        return self.getCalibratedValue(S_H_B_CAL, NIR)

    def getCalibratedT(self):
        return self.getCalibratedValue(T_I_C_CAL, NIR)

    def getCalibratedU(self):
        return self.getCalibratedValue(U_J_D_CAL, NIR)

    def getCalibratedV(self):
        return self.getCalibratedValue(V_K_E_CAL, NIR)

    def getCalibratedW(self):
        return self.getCalibratedValue(W_L_F_CAL, NIR)

    #Given an address, read four bytes and return the floating point calibrated value
    def getCalibratedValue(self, calAddress, device):
        self.selectDevice(device)

        b0 = self.virtualReadRegister(calAddress + 0)
        b1 = self.virtualReadRegister(calAddress + 1)
        b2 = self.virtualReadRegister(calAddress + 2)
        b3 = self.virtualReadRegister(calAddress + 3)

        #Channel calibrated values are stored big-endian
        calBytes = 0
        calBytes |= (b0 << (8 * 3))
        calBytes |= (b1 << (8 * 2))
        calBytes |= (b2 << (8 * 1))
        calBytes |= (b3 << (8 * 0))

        return self.convertBytesToFloat(calBytes)

    #Given 4 bytes returns the floating point value
    def convertBytesToFloat(self, value):
        b = struct.pack('=L', value)
        f = struct.unpack('f', b)
        return f[0]

    def setMeasurementMode(self, mode):
        mode = 0b11 if mode > 0b11 else mode

        value = self.virtualReadRegister(CONFIG)
        value &= 0b11110011
        value |= (mode << 2)
        self.virtualWriteRegister(CONFIG, value)

    def setGain(self, gain):
        gain = 0b11 if gain > 0b11 else gain

        value = self.virtualReadRegister(CONFIG)
        value &= 0b11001111
        value |= (gain << 4)
        self.virtualWriteRegister(CONFIG, value)

    def setIntegrationCycles(self, cycleValue):
        self.virtualWriteRegister(INTEGRATION_TIME, cycleValue)

    def enableInterrupt(self):
        value = self.virtualReadRegister(CONFIG)
        value |= 0b01000000
        self.virtualWriteRegister(CONFIG, value)

    def disableInterrupt(self):
        value = self.virtualReadRegister(CONFIG)
        value &= 0b10111111
        self.virtualWriteRegister(CONFIG, value)

    def dataAvailable(self):
        value = self.virtualReadRegister(CONFIG)
        return value & 0x02

    def enableBulb(self, device):
        self.selectDevice(device)

        value = self.virtualReadRegister(LED_CONFIG)
        value |= 0b00001000
        self.virtualWriteRegister(LED_CONFIG, value)

    def disableBulb(self, device):
        self.selectDevice(device)

        value = self.virtualReadRegister(LED_CONFIG)
        value &= 0b11110111
        self.virtualWriteRegister(LED_CONFIG, value)

    def setBulbCurrent(self, current, device):
        self.selectDevice(device)
        current = 0b11 if current > 0b11 else current
        value = self.virtualReadRegister(LED_CONFIG)
        value &= 0b11001111
        value |= (current << 4)
        self.virtualWriteRegister(LED_CONFIG, value)

    def selectDevice(self, device):
        self.virtualWriteRegister(DEV_SELECT_CONTROL, device)

    def enableIndicator(self):
        value = self.virtualReadRegister(LED_CONFIG)
        value |= 0b00000001

        self.selectDevice(NIR)
        self.virtualWriteRegister(LED_CONFIG, value)

    def disableIndicator(self):
        value = self.virtualReadRegister(LED_CONFIG)
        value &= 0b11111110

        self.selectDevice(NIR)
        self.virtualWriteRegister(LED_CONFIG, value)

    def setIndicatorCurrent(self, current):
        current = 0b11 if current > 0b11 else current
        value = self.virtualReadRegister(LED_CONFIG)
        value &= 0b11111001
        value |= (current << 1)

        self.selectDevice(NIR)
        self.virtualWriteRegister(LED_CONFIG, value)

    def getTemperature(self, deviceNumber):
        self.selectDevice(deviceNumber)
        return self.virtualReadRegister(DEVICE_TEMP)

    def getTemperatureAverage(self):
        average = 0
        for x in range(3):
            average += self.getTemperature(x)
        return float(average) / 3

    def softReset(self):
        value = self.virtualReadRegister(CONFIG)
        value |= 0x80
        self.virtualWriteRegister(CONFIG, value)

    def virtualReadRegister(self, virtualAddr):
        status = self.readRegister(STATUS_REG)
        if (status & RX_VALID) != 0:
            incoming = self.readRegister(READ_REG)

        while(1):
            status = self.readRegister(STATUS_REG)
            if (status & TX_VALID) == 0:
                break
            time.sleep(POLLING_DELAY)

        self.writeRegister(WRITE_REG, virtualAddr)

        while(1):
            status = self.readRegister(STATUS_REG)
            if (status & RX_VALID) != 0:
                break
            time.sleep(POLLING_DELAY)

        incoming = self.readRegister(READ_REG)
        return incoming

    def virtualWriteRegister(self, virtualAddr, dataToWrite):
        while(1):
            status = self.readRegister(STATUS_REG)
            if (status & TX_VALID) == 0:
                break
            time.sleep(POLLING_DELAY)

        self.writeRegister(WRITE_REG, virtualAddr | 0x80)

        while(1):
            status = self.readRegister(STATUS_REG)
            if (status & TX_VALID) == 0:
                break
            time.sleep(POLLING_DELAY)

        self.writeRegister(WRITE_REG, dataToWrite)

    def readRegister(self, addr):
        return self._bus.read_byte_data(I2C_ADDR, addr)

    def writeRegister(self, addr, val):
        return self._bus.write_byte_data(I2C_ADDR, addr, val)

