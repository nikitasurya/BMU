import commands as cmd #import commands definition from another file

from time import sleep
from smbus import SMBus

class BMU(object):
    
    def __init__(self, deviceAdd =0x0B, busnum = 1):
        """
        Initalise SMBus bus 1
        Returns: nothing
        """
        self.deviceAdd = deviceAdd
        if (busnum != 1 or busnum != 2):
            print "Error, SMbus needs to be either 1 or 2: Initialising with SMBus 1"
            self.bus1 = SMBus(1)
        else:
            self.bus1 = SMBus(busnum)
    
    """------------Voltage Reading------------------------"""
    def cell_voltage(self, num): 
        """
        Reads voltage of cell
        Requires Cell Number 1 - 4
        Returns value in mV
        """
        if ((num!= 1) and (num!=2) and (num!=3) and (num!=4)):
            print "Error: Cell number not valid"
            return -1

        if (num == 1):
            temp = self.smbusRead(cmd.cellVoltage1)
        if (num == 2):
            temp = self.smbusRead(cmd.cellVoltage2)
        if (num == 3):
            temp = self.smbusRead(cmd.cellVoltage3)
        if (num == 4):
            temp = self.smbusRead(cmd.cellVoltage4)

        return ((temp[1]<<8) | temp[0])

    def totalCellVoltage(self):
        """
        Reads individual cell and sum it up
        Returns total cell voltage in mV
        """
        temp = self.smbusRead(cmd.voltage)
        total_voltage= temp[1]<<8 | temp[0]
        return total_voltage

    """------------Current Reading------------------------"""
    def currentDraw(self):
        """
        Reads the current draw
        returns current draw in mA
        """
        temp = self.smbusRead(cmd.current)
        current = temp[1]<<8 | temp[0]
        #2 complement number to represent negative numbers
        if (current & 0x8000):
            current = current - 0x10000

        return current

    def averageCurrentDraw(self):
        """
        Reads the current draw
        returns current draw in mA
        """
        temp = self.smbusRead(cmd.averageCurrent)
        aveCurrent = temp[1]<<8 | temp[0]
        #2 complement number to represent negative numbers
        if (aveCurrent & 0x8000):
            aveCurrent = aveCurrent - 0x10000

        return aveCurrent

    """------------LEDCONTROL-----------------------------"""

    def toggleLED(self):
        """
        Toggle LEDs
        Returns nothing
        """
        val = [0x01<<1, (cmd.LEDToggle & 0xFF), ((cmd.LEDToggle>>8) & 0xFF)]
        self.smbusWrite(val)

    """------------Mosfet CONTROL-------------------------"""
    def toggle_CHG_FET(self):
        """
        toggles the CHG FET 
        Returns nothing
        """
        val1=[0x01<<1, cmd.CHGFetToggle , 0x00]
        self.smbusWrite(val1)
        
    def toggle_DCHG_FET(self):
        """
        toggles the discharge FET
        Returns nothing
        """
        val2=[0x01<<1, cmd.DSGFetToggle , 0x00]
        self.smbusWrite(val2)
    
    """------------Temperature Reading -------------------"""
    def temp_read (self):
        """ 
        Reads the temperature of the cell
        Returns the value in K
        """
        cell_temperature= self.smbusRead(cmd.temperature)
        total_temp=((cell_temperature[1]<<8)|(cell_temperature[0]))/10  #divided by 10 because temperature is in 0.1K unit
        return total_temp
   
    """-------------Battery Status ------------------------"""
    #TODO check it again when we do a qualified discharge
    def remainingCapacity(self):
        """
        Reads remaining capacity of the battery
        If BatteryMode()[CAPM] = 0, then the data reports in mAh.
        If BatteryMode()[CAPM] = 1, then the data reports in 10 mWh.
        """
        temp = self.smbusRead(cmd.remainingCapacity)
        return ((temp[1] << 8) | temp[0])

    """-------------Battery Info ------------------------"""
    def fullChargeCapacity(self):
        """
        Predicted battery capacity when fully charged
        """
        temp = self.smbusRead(cmd.fullChargeCapacity)
        return ((temp[1] << 8) | temp[0])

    """ ---------------Remaining time alarm----------------"""
    #To check if its a read or write
    def time_remaining(self):
        """
        by default sets the alarm when
        battery will finish in 10 minutes
        """
        val3=[0x01<<1, cmd.remainingTimeAlarm, 0x00]
        self.smbusWrite(val3)
        
    """-------------Perment Failure Inidcation-------------"""
    def permanent_failure(self):
        """
        enable/disables permanent
        failure protections
        """
        val5=[0x01<<1, cmd.permanentFailure, 0x00]
        self.smbusWrite(val5)

    """-------------Status Registers ----------------------"""
    """ Reads the status registers of the name of the method
        Returns 32 bit value unless otherwise stated"""
    def operationStatusReg(self): 
        temp = self.smbusRead(cmd.operationStatus)
        return ((temp[3] << 24) | (temp[2] << 16) | (temp[1] << 8) | temp[0])

    def gaugingStatusReg(self):   
        """ Return 16 bit"""
        temp = self.smbusRead(cmd.gaugingStatus)
        return ((temp[1] << 8) | temp[0])

    def safetyAlertReg(self):
        temp = self.smbusRead(cmd.safetyAlert)
        return ((temp[3] << 24) | (temp[2] << 16) | (temp[1] << 8) | temp[0])

    def safetyStatusReg(self):
        temp = self.smbusRead(cmd.safetyStatus)
        return ((temp[3] << 24) | (temp[2] << 16) | (temp[1] << 8) | temp[0])

    def PFStatusReg(self):
        temp = self.smbusRead(cmd.PFStatus)
        return ((temp[3] << 24) | (temp[2] << 16) | (temp[1] << 8) | temp[0])

    def PFAlertReg(self):
        temp = self.smbusRead(cmd.PFAlert)
        return ((temp[3] << 24) | (temp[2] << 16) | (temp[1] << 8) | temp[0])

    def chargingStatusReg(self):
        temp = self.smbusRead(cmd.chargingStatus)
        return ((temp[3] << 24) | (temp[2] << 16) | (temp[1] << 8) | temp[0])
    
    def manufacturingStatusReg(self):
        """Returns 16 bit"""
        temp = self.smbusRead(cmd.manufacturingStatus)
        return ((temp[1] << 8) | temp[0])


        """-------------Helper Function------------------------"""

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

    bmu = BMU(0x0B, 1)

    while True:
        print "Important values"
        bmu.toggleLED()
        bmu.toggle_DCHG_FET()
        print "Cell 1:", bmu.cell_voltage(1)
        print "Cell 2:", bmu.cell_voltage(2)
        print "Cell 3:", bmu.cell_voltage(3)
        print "total cell voltage:", bmu.totalCellVoltage()
        print "Temp:", bmu.temp_read()
        print "Current", bmu.currentDraw()
        print "Average Current", bmu.averageCurrentDraw()
        print "Remaining Cap", bmu.remainingCapacity()
        print "Full Charge Cap", bmu.fullChargeCapacity()
        sleep(1)
