import commands as cmd #import commands definition from another file

from time import sleep
from smbus import SMBus

class BMU(self, deviceAdd):
    
    def __init__(self, busnum = 1):
        """
        Initalise SMBus bus 1
        Returns: nothing
        """
        self.deviceAdd = deviceAdd
        self.bus1 = SMBus(1)

    def readCell(self, cellNum):
        """
        Reads the voltage of the cell
        Returns voltage in mV
        """
        self.bus1.read_i2c_block_data(self.deviceAdd,cmd.readcellcommand)
       

    def totalCellVoltage(self):
        """
        Reads individual cell and sum it up
        Returns total cell voltage in mV
        """
        temp = self.smbusRead(cmd.voltage)
        #TODO check if correct
        return temp[1]<<8 | temp[0]
 
    def toggleLED(self):
        """
        Toggle LEDs
        Returns nothing
        """
        val = [0x01<<1, (cmd.LEDToggle & 0xFF), ((cmd.LEDToggle>>8) & 0xFF)]
        smbusWrite(vale)

    def smbusRead(self, value):
        """
        Helper function to read smbus
        returns block of data read from smbus
        """
        return self.bus1.read_i2c_block_data(self.deviceAdd, value)

    def smbusWrite(self, value):
        """
        Helper function to write to smbus
        Returns nothing
        """
        self.bus1.write_i2c_block_data(self.deviceAdd, cmd.manBlockAcc, value)


if __name__ == '__main__':

    bmu = BMU()

    while True:
        print "Important values"
        bmu.totalCellVoltage()
        sleep(1)
