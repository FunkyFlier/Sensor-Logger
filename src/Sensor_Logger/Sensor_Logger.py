'''
Created on Jan 26, 2016

@author: Michael Baker
'''

import Adafruit_GPIO.FT232H as FT232H
import Adafruit_GPIO as GPIO
import struct
import ctypes
import time
from time import localtime, strftime
import serial
#import logging

# Temporarily disable FTDI serial drivers.
FT232H.use_FT232H()
# Find the first FT232H device.
ft232h = FT232H.FT232H()
dateString = strftime("dataSet_%d_%m_%Y_%H-%M.csv", localtime())
print dateString
#fileName = open("dataSet"+str(time.clock())+".csv", 'w')
fileName = open(dateString, 'w')
#logging.basicConfig(filename='dataSet1.csv',format='',level=logging.INFO)
ft232h.setup(6, GPIO.OUT)  
ft232h.setup(7, GPIO.OUT)  

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
        gyroFloatList = [0,0,0]
        gyroFloatList[0] = gyroIntList[0] * 0.07
        gyroFloatList[1] = gyroIntList[1] * 0.07
        gyroFloatList[2] = gyroIntList[2] * 0.07
        return gyroFloatList;
    
class LSM303DLHMagDriver:
    #mag defines ST HMC5983DLHC - will work with the HMC5883L
    MAG_ADDRESS      = 0x1E  #0001 1110
    HMC5983_CRA_REG  = 0x00
    HMC5983_CRB_REG  = 0x01
    HMC5983_MR_REG   = 0x02
    HMC5983_OUT_X_H  = 0x03
    HMC5983_OUT_X_L  = 0x04
    HMC5983_OUT_Z_H  = 0x05
    HMC5983_OUT_Z_L  = 0x06
    HMC5983_OUT_Y_H  = 0x07
    HMC5983_OUT_Y_L  = 0x08
    
    HMC5983_ID_A     = 0x0A
    HMC5983_ID_B     = 0x0B
    HMC5983_ID_C     = 0x0C
    HMC5983_WHO_AM_I = 0x0F
    READ = 0x80
    
    biasX = 17.405503
    biasY = -73.633369
    biasZ = -54.624221
    w00 = 1.001680
    w01 = 0.075379
    w02 = -0.043184
    w10 = 0.075379
    w11 = 1.003391
    w12 = 0.034361
    w20 = -0.043184
    w21 = 0.034361
    w22 = 1.015037
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
        magFloatList = [0,0,0]
        magFloatList[0] = magIntList[0] * self.w00 + magIntList[1] * self.w01 + magIntList[2] * self.w02 
        magFloatList[1] = magIntList[0] * self.w10 + magIntList[1] * self.w11 + magIntList[2] * self.w12 
        magFloatList[2] = magIntList[0] * self.w20 + magIntList[1] * self.w21 + magIntList[2] * self.w22 
        return magFloatList    
    
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
    scaleFactor = 9.8/4096
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
        accFloatList = [0,0,0]
        accFloatList[0] = accIntList[0] * self.scaleFactor
        accFloatList[1] = accIntList[1] * self.scaleFactor
        accFloatList[2] = accIntList[2] * self.scaleFactor
        return accFloatList    

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
    
    MS5611_BARO_CONV_TIME     = 11
    C = []
    pressure = 0.0
    pollTimer = 0
    pollState = 0
    D2 = ctypes.c_uint32(0).value
    D1 = ctypes.c_uint32(0).value
    newBaroData = False
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
                self.newBaroData = True
                #print self.pressure
                self.pollState = 0
                
class UBLOXPVTParser:
    ubloxState  = 0
    inByte      = 0x00
    summingByte = 0x00

    sumRcvdA    = 0x00
    sumRcvdB    = 0x00
 
    sumCalcA    = 0x00
    sumCalcB    = 0x00
    newGPSData = False
    numSats = []
    longitude = []
    lattitude = []
    heightEllipsoid = []
    heightMSL = []
    velN = []
    velE = []
    velD = []
    ublox = []
    ubloxList = []
    def __init__(self):
        self.ublox = serial.Serial('COM15',38400)
        self.ublox.close()
        self.ublox.open()
        self.ubloxState  = 0
    def GetGPSByte(self):
        inList = self.ublox.read(1)
        gpsByte = struct.unpack('B',inList[0:1])
        return gpsByte[0]       
    def Poll(self):
        while self.ublox.inWaiting() > 0:
            if self.ubloxState == 0:#check first byte
                self.inByte = self.GetGPSByte()
                if self.inByte == 0xB5:
                    self.ubloxState = 1    
            elif self.ubloxState == 1:#check second byte
                self.inByte = self.GetGPSByte()
                if self.inByte == 0x62:
                    self.ubloxState = 2
                else:
                    self.ubloxState = 0
                self.sumCalcA = 0x00
                self.sumCalcB = 0x00
            elif self.ubloxState == 2:#check message type
                self.inByte = self.GetGPSByte()
                self.sumCalcA += self.inByte
                self.sumCalcB += self.sumCalcA
                if self.inByte == 0x01:
                    self.ubloxState = 3
                else:
                    self.ubloxState = 0
            elif self.ubloxState == 3:#check message number
                self.inByte = self.GetGPSByte()
                self.sumCalcA += self.inByte
                self.sumCalcB += self.sumCalcA
                if self.inByte == 0x07:
                    self.ubloxState = 4
                else:
                    self.ubloxState = 0
            elif self.ubloxState == 4:#get packet length
                if self.ublox.inWaiting() >= 2:
                    lengthList = self.ublox.read(2)
                    packetLength = struct.unpack('H',lengthList[0:2])
                    
                    inByteList = struct.unpack('B',lengthList[0:1])
                    self.sumCalcA += inByteList[0]
                    self.sumCalcB += self.sumCalcA
                    
                    inByteList = struct.unpack('B',lengthList[1:2])
                    self.sumCalcA += inByteList[0]
                    self.sumCalcB += self.sumCalcA
                    #print(packetLength)
                    if packetLength[0] == 92:
                        self.ubloxState = 5
                    else:
                        self.ubloxState = 0
            elif self.ubloxState == 5:#get GPS packet
                if self.ublox.inWaiting() >= 92:
                    #print "5"
                    self.ubloxList = self.ublox.read(92)
                    self.ubloxState = 6
            elif self.ubloxState == 6:#get first sum
                self.sumRcvdA = self.GetGPSByte()
                self.ubloxState = 7
            elif self.ubloxState == 7:#get second sum then generate and check
                self.sumRcvdB = self.GetGPSByte()
                for i in range(0,92):
                    summingByte = struct.unpack('B',self.ubloxList[i:i+1])
                    self.sumCalcA += summingByte[0]
                    self.sumCalcB += self.sumCalcA
                self.sumCalcA = self.sumCalcA & 0xFF
                self.sumCalcB = self.sumCalcB & 0xFF
                if (self.sumCalcA == self.sumRcvdA) and (self.sumCalcB == self.sumRcvdB):
                    #unpack the data from the list
                    ubloxTouple = struct.unpack('LHBBBBBBLlBBBBllllLLlllllLLH',self.ubloxList[0:78])
                    self.numSats = ubloxTouple[13]
                    self.longitude = ubloxTouple[14]
                    self.lattitude = ubloxTouple[15]
                    self.heightEllipsoid = ubloxTouple[16]
                    self.heightMSL = ubloxTouple[17]
                    self.velN = ubloxTouple[20]
                    self.velE = ubloxTouple[21]
                    self.velD = ubloxTouple[22]
                    self.newGPSData = True
                    #print ubloxTouple
                    #print (self.numSats,self.longitude,self.lattitude,self.heightEllipsoid,self.heightMSL,self.velN,self.velE,self.velD)
                self.ubloxState = 0
#end classes ---------------------------------------------------------    

 
ClockStart()   

gyro = L3G4200DDriver()
acc = LSM303DLHAccDriver()
mag = LSM303DLHMagDriver()
baro = MS56XXDriver()
gps = UBLOXPVTParser()

mag.Setup()
gyro.Setup()
acc.Setup()
baro.Setup()

print gyro.Read()
print acc.Read()
print mag.Read()
highRateTimer = micros()
lowRateTimer = millis()
accGyroString = []
magString = []
gpsString = []
pressureString = []
now = micros()
while True:
    if (micros() - highRateTimer) > 1250:
        highRateTimer = micros()
        gyroList = gyro.Read()
        accList = acc.Read()
        accGyroString = "%f,%i,%f,%f,%f,%f,%f,%f"%(micros(),0
                                                ,gyroList[0],gyroList[1]
                                                ,gyroList[2],accList[0]
                                                ,accList[1],accList[2])
        #logging.info(accGyroString)
        fileName.write(accGyroString)
        #print accGyroString
    if (micros() - lowRateTimer) > 13333.333:
        ft232h.output(7, GPIO.HIGH)
        lowRateTimer = micros()
        magList = mag.Read()
        magString = "%f,%i,%f,%f,%f"%(micros(),1,magList[0],magList[1],magList[2])
        fileName.write(magString)
        #logging.info(magString)
    baro.Poll()
    if baro.newBaroData == True:
        baro.newBaroData = False
        pressureString = "%f,%i,%i"%(micros(),2,baro.pressure)
        #logging.info(pressureString)
        fileName.write(pressureString)
    gps.Poll()
    if gps.newGPSData == True:
        gps.newGPSData = False
        gpsString = "%f,%i,%i,%i,%i,%i,%i,%i,%i,%i"%(micros(),3,
                                                     gps.numSats,
                                                     gps.longitude,
                                                     gps.lattitude,
                                                     gps.heightEllipsoid,
                                                     gps.heightMSL,
                                                     gps.velN,
                                                     gps.velE,
                                                     gps.velD)
        #logging.info(gpsString)
        fileName.write(gpsString)
        #print gpsString


















