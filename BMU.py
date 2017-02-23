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

    def totalCellVoltage(self):
        """
        Reads individual cell and sum it up
        Returns total cell voltage in mV
        """
        temp = self.smbusRead(cmd.voltage)
        total_voltage= temp[1]<<8 | temp[0]
        return total_voltage
 
   def toggleLED(self):
        """
        Toggle LEDs
        Returns nothing
        """
        val = [0x01<<1, (cmd.LEDToggle & 0xFF), ((cmd.LEDToggle>>8) & 0xFF)]
        smbusWrite(val)

    def toggle_CHG_FET(self):
        """
        toggles the CHG FET 
        Returns value to write for toggle
        """
        val1=[0x01<<1, cmd.CHGFET_Toggle , 0x00]
        smbusWrite(val1)
        
    def toggle_DCHG_FET(self):
        """
        toggles the discharge FET
        Returns value to write for toggle 
        """
        val2=[0x01<<1, cmd.DCHGFET_Toggle , 0x00]
        smbusWrite(val2)
        
    def temp_read (self):
        """ 
        Reads the temperature of the cell
        Returns the value in K
        """
        cell_temperature= self.smbusRead(cmd.temperature)
        total_temp=((cell_temperature[1]<<8)|(cell_temperature[0]))/10  #divided by 10 because temperature is in 0.1K unit
        return total_temp
    
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
