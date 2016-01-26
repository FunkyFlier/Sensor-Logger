'''
Created on Jan 26, 2016

@author: work
'''

import Adafruit_GPIO.FT232H as FT232H
import struct

# Temporarily disable FTDI serial drivers.
FT232H.use_FT232H()
# Find the first FT232H device.
ft232h = FT232H.FT232H()

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
        self.gyro.write8(self.L3G_CTRL_REG1, 0x8F)     

    def Read(self):
        gyroByteList = self.gyro.readList(self.L3G_OUT_X_L | self.READ, 6)
        gyroIntList = struct.unpack('hhh',gyroByteList[0:6])
        return gyroIntList;
    
    
gyro = L3G4200DDriver()

gyro.Setup()

print gyro.Read()