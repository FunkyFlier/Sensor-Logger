'''
Created on Jan 26, 2016

@author: work
'''

import Adafruit_GPIO.FT232H as FT232H
import struct
import ctypes
import time
from math import floor
# Temporarily disable FTDI serial drivers.
FT232H.use_FT232H()
# Find the first FT232H device.
ft232h = FT232H.FT232H()


def millis():
    return time.clock() * 1000
    #return floor(time.clock() * 1000)
def micros():
    return time.clock() * 1000000
    #return floor(time.clock() * 1000000)
def nanos():
    return time.clock() * 1000000000
    #return floor(time.clock() * 1000000000)
def ClockStart():
    millis()
    micros()
    nanos()
class L3G4200DDriver:
    #gyro defines - ST L3G2
    L3G_CTRL_REG1 = 0x20
    L3G_CTRL_REG2 = 0x21
    L3G_CTRL_REG3 = 0x22
    L3G_CTRL_REG4 = 0x23
    L3G_CTRL_REG5 = 0x24
    L3G_OUT_X_L   = 0x28
    L3G_WHO_AM_I  = 0x0F
    L3G_I2C_ADDR  = 0x69#0110 1001   
    READ = 0x80
 
    def __init__(self):

        self.gyro = FT232H.I2CDevice(ft232h,self.L3G_I2C_ADDR,400000)
        
    def Setup(self):
        #whoAmI = gyro.readU8(L3G_WHO_AM_I)
        #print format(whoAmI,'02x')
        #print (bin(whoAmI))
        self.gyro.write8(self.L3G_CTRL_REG2, 0x00)
        self.gyro.write8(self.L3G_CTRL_REG3, 0x00)
        self.gyro.write8(self.L3G_CTRL_REG4, 0x20)
        self.gyro.write8(self.L3G_CTRL_REG5, 0x02)
        self.gyro.write8(self.L3G_CTRL_REG1, 0xCF)     
        #800 Hz 30Hz Lpf cut off
    def Read(self):
        gyroByteList = self.gyro.readList(self.L3G_OUT_X_L | self.READ, 6)
        gyroIntTouple = struct.unpack('hhh',gyroByteList[0:6])
        gyroIntList = list(gyroIntTouple)
        return gyroIntList;
    
class LSM303DLHMagDriver:
    #mag defines ST HMC5983DLHC - will work with the HMC5883L
    MAG_ADDRESS     = 0x1E  #0001 1110
    HMC5983_CRA_REG = 0x00
    HMC5983_CRB_REG = 0x01
    HMC5983_MR_REG  = 0x02
    HMC5983_OUT_X_H = 0x03
    HMC5983_OUT_X_L = 0x04
    HMC5983_OUT_Z_H = 0x05
    HMC5983_OUT_Z_L = 0x06
    HMC5983_OUT_Y_H = 0x07
    HMC5983_OUT_Y_L = 0x08
    
    HMC5983_ID_A     = 0x0A
    HMC5983_ID_B     = 0x0B
    HMC5983_ID_C     = 0x0C
    HMC5983_WHO_AM_I = 0x0F
    READ = 0x80
    def __init__(self):
        self.mag  = FT232H.I2CDevice(ft232h,self.MAG_ADDRESS,400000)
    def Setup(self):
        self.mag.write8(self.HMC5983_CRA_REG, 0x18)
        self.mag.write8(self.HMC5983_CRB_REG, 0x60)
        self.mag.write8(self.HMC5983_MR_REG, 0x00)
    def Read(self):
        magByteList = self.mag.readList(self.HMC5983_OUT_X_H | self.READ,6)
        magIntTouple = struct.unpack('>hhh',magByteList[0:6])
        magIntList = list(magIntTouple)
        return magIntList    
    
class LSM303DLHAccDriver:
    #ACC defines
    ACC_ADDRESS       = 0x18#0001 1000
    CTRL_REG1_A       = 0x20
    CTRL_REG2_A       = 0x21
    CTRL_REG3_A       = 0x22
    CTRL_REG4_A       = 0x23
    CTRL_REG5_A       = 0x24
    CTRL_REG6_A       = 0x25
    HP_FILTER_RESET_A = 0x25
    REFERENCE_A       = 0x26
    STATUS_REG_A      = 0x27
    
    OUT_X_L_A         = 0x28
    OUT_X_H_A         = 0x29
    OUT_Y_L_A         = 0x2A
    OUT_Y_H_A         = 0x2B
    OUT_Z_L_A         = 0x2C
    OUT_Z_H_A         = 0x2D
    
    READ = 0x80
    
    def __init__(self):
        self.acc = FT232H.I2CDevice(ft232h,self.ACC_ADDRESS,400000) 
    
    def Setup(self):
        self.acc.write8(self.CTRL_REG1_A, 0x3F)
        self.acc.write8(self.CTRL_REG2_A, 0x00)
        self.acc.write8(self.CTRL_REG3_A, 0x00)
        self.acc.write8(self.CTRL_REG4_A, 0x30)
        self.acc.write8(self.CTRL_REG5_A, 0x00)
        #1kHz +/- 8G
    def Read(self):
        accByteList = self.acc.readList(self.OUT_X_L_A | self.READ, 6)
        accIntTouple = struct.unpack('hhh',accByteList[0:6])
        accIntList = list(accIntTouple)
        return accIntList    

class MS56XXDriver:
    BARO_ADDR                 = 0x76 
    MS5611_RESET              = 0x1E
    MS5611_PROM_Setup         = 0xA0
    MS5611_PROM_C1            = 0xA2
    MS5611_PROM_C2            = 0xA4
    MS5611_PROM_C3            = 0xA6
    MS5611_PROM_C4            = 0xA8
    MS5611_PROM_C5            = 0xAA
    MS5611_PROM_C6            = 0xAC
    MS5611_PROM_CRC           = 0xAE
    MS5611_CONVERT_D1_OSR4096 = 0x48   
    MS5611_CONVERT_D2_OSR4096 = 0x58   
    
    MS5611_ADC_READ           = 0x00
    
    MS5611_BARO_CONV_TIME     = 10
    C = []
    pressure = 0.0
    pollTimer = 0
    pollState = 0
    D2 = ctypes.c_uint32(0).value
    D1 = ctypes.c_uint32(0).value
    def __init__(self):
        self.baro = FT232H.I2CDevice(ft232h,self.BARO_ADDR,400000)
        
    def Setup(self): 
        baroByteList = self.baro.readList(self.MS5611_PROM_Setup , 2)
        self.C.append(ctypes.c_uint16(struct.unpack('>H',baroByteList[0:2])[0]).value)
           
        baroByteList = self.baro.readList(self.MS5611_PROM_C1 , 2)
        self.C.append(ctypes.c_uint16(struct.unpack('>H',baroByteList[0:2])[0]).value)
              
        baroByteList = self.baro.readList(self.MS5611_PROM_C2, 2)
        self.C.append(ctypes.c_uint16(struct.unpack('>H',baroByteList[0:2])[0]).value)
              
        baroByteList = self.baro.readList(self.MS5611_PROM_C3, 2)
        self.C.append(ctypes.c_uint16(struct.unpack('>H',baroByteList[0:2])[0]).value)
              
        baroByteList = self.baro.readList(self.MS5611_PROM_C4, 2)
        self.C.append(ctypes.c_uint16(struct.unpack('>H',baroByteList[0:2])[0]).value)
              
        baroByteList = self.baro.readList(self.MS5611_PROM_C5, 2)
        self.C.append(ctypes.c_uint16(struct.unpack('>H',baroByteList[0:2])[0]).value)
              
        baroByteList = self.baro.readList(self.MS5611_PROM_C6, 2)
        self.C.append(ctypes.c_uint16(struct.unpack('>H',baroByteList[0:2])[0]).value)
        self.pollTimer = millis()
        self.pollState = 0
        print self.C
       
        
    def Poll(self):
        if self.pollState == 0:#send D2 convert
            self.baro.writeRaw8(self.MS5611_CONVERT_D2_OSR4096)
            self.pollState = 1
        elif self.pollState == 1:#
            if (millis() - self.pollTimer) >  self.MS5611_BARO_CONV_TIME:
                self.pollTimer = millis()
                adcList = self.baro.readList(self.MS5611_ADC_READ, 3)
                adcList.insert(0,'\00')
                self.D2 = struct.unpack('>L',adcList[0:4])[0]
                self.pollState = 2
        elif self.pollState == 2:
            self.baro.writeRaw8(self.MS5611_CONVERT_D1_OSR4096)
            self.pollState = 3
        elif self.pollState == 3:
            if (millis() - self.pollTimer) >  self.MS5611_BARO_CONV_TIME:
                self.pollTimer = millis()
                adcList = self.baro.readList(self.MS5611_ADC_READ, 3)
                adcList.insert(0,'\00')
                self.D1 = struct.unpack('>L',adcList[0:4])[0]
                dT = ctypes.c_int32(self.D2-self.C[5]*pow(2,8)).value
                TEMP = ctypes.c_float((2000+(dT*self.C[6])/pow(2,23))).value
                OFF = ctypes.c_float(self.C[2]*pow(2,17)+dT*self.C[4]/pow(2,6)).value
                SENS = ctypes.c_float(self.C[1]*pow(2,16)+dT*self.C[3]/pow(2,7)).value
                # perform higher order corrections
                T2=ctypes.c_float(0).value
                OFF2=ctypes.c_float(0).value 
                SENS2=ctypes.c_float(0).value
                if TEMP < 2000:
                    T2=ctypes.c_float(dT*dT/pow(2,31)).value
                    OFF2=ctypes.c_float(61*(TEMP-2000)*(TEMP-2000)/pow(2,4)).value
                    SENS2=ctypes.c_float(2*(TEMP-2000)*(TEMP-2000)).value
                    if TEMP < -1500:
                        OFF2+=ctypes.c_float(15*(TEMP+1500)*(TEMP+1500)).value
                        SENS2+=ctypes.c_float(8*(TEMP+1500)*(TEMP+1500)).value 
                TEMP -= T2
                OFF -= OFF2
                SENS -= SENS2
                self.pressure = ctypes.c_float((((self.D1*SENS)/pow(2,21)-OFF)/pow(2,15))).value
                time.sleep(1)
                print self.pressure
                self.pollState = 0
                
#end classes ---------------------------------------------------------     
ClockStart()   
gyro = L3G4200DDriver()
acc = LSM303DLHAccDriver()
mag = LSM303DLHMagDriver()
baro = MS56XXDriver()

mag.Setup()
gyro.Setup()
acc.Setup()
baro.Setup()

print gyro.Read()
print acc.Read()
print mag.Read()
 
while True:
    baro.Poll()





















