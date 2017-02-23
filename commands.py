'''
SBS Commands
Manufacture block provides a method of reading and 
writing data in the manufacture access system(MAC)

Example: To turn on Discharge Fet
Send to Device Address = 0x0B
Send command to Register  = 0x44
Send data [0x01<<1, 0x20, 0x00] (data is in little endian form and addr is 7 bit hence the left shift)
'''
manufactureBlockAccess      = 0x44
manBlockAcc                 = 0x44
manufactureAccesss          = 0x0000

deviceType                  = 0x0001
firmwareVersion             = 0x0002
hardwareVersion             = 0x0003
instructionFlashSignature   = 0x0004
staticDFSignature           = 0x0005
allDFSignature              = 0x0009
shutdownMode                = 0x0010
sleepMode                   = 0x0011

"""Mosfet Control"""
fuseToggle                  = 0x001D
PCHGFetToggle               = 0x001E
CHGFetToggle                = 0x001F
DSGFetToggle                = 0x0020
fetControl                  = 0x0022

lifetimeDataCollection      = 0x0023
permanentFailure            = 0x0024
blackBoxRecorder            = 0x0025
fuse                        = 0x0026
LEDDisplayEnable            = 0x0027
lifetimeDataReset           = 0x0028
permanentFailDataReset      = 0x0029
blackBoxRecorderReset       = 0x002A

"""LED Control"""
LEDToggle                   = 0x002B
LEDDisplayPress             = 0x002C

calibrationMode             = 0x002D
lifetimeDataFLush           = 0x002E
lifetimeDataSpeedUpMode     = 0x002F
sealDevice                  = 0x0030
securityKeys                = 0x0035
authenticationKey           = 0x0037

deviceReset                 = 0x0041
safetyAlert                 = 0x0050
safetyStatus                = 0x0051
PFAlert                     = 0x0052
PFStatus                    = 0x0053
operationStatus             = 0x0054
chargingStatus              = 0x0055
gaugingStatus               = 0x0056
manufacturingStatus         = 0x0057
AFERegister                 = 0x0058
lifetimeDataBlock1          = 0x0060
lifetimeDataBlock2          = 0x0061
lifetimeDataBlock3          = 0x0062
lifetimeDataBlock4          = 0x0063
lifetimeDataBlock5          = 0x0064
manufactureInfo             = 0x0070
DAStatus1                   = 0x0071
DAStatus2                   = 0x0072
manufactureInfo2            = 0x007A
ROMMode                     = 0x0F00
"""
Data Flash Access 0x4000-0x5FFF
"""
exitCalibrationMode         = 0xF080
outputCCADC                 = 0xF081
outputShortedCCADCCal       = 0xF082


remainingCapacityAlarm      = 0x01
remainingTimeAlarm          = 0x02
batteryMode                 = 0x03
atRate                      = 0x04
atRateTimeToFull            = 0x05
atRateTimeToEmpty           = 0x06
atRateOK                    = 0x07
temperature                 = 0x08
voltage                     = 0x09
current                     = 0x0A
averageCurrent              = 0x0B
maxError                    = 0x0C
relativeStateOfCharge       = 0x0D
absoluteStateOfCharge       = 0x0E
remainingCapacity           = 0x0F
fullChargeCapacity          = 0x10
runTimeToEmpty              = 0x11
averageTimeToEmpty          = 0x12
averageTimeToFull           = 0x13
chargingCurrent             = 0x14
chargingVoltage             = 0x15
batteryStatus               = 0x16
cycleCount                  = 0x17
designCapacity              = 0x18
designVoltage               = 0x19
specificationInfo           = 0x1A
manufactureDate             = 0x1B
serialNumber                = 0x1C
manufactureName             = 0x20
deviceName                  = 0x21
deviceChemistry             = 0x22
manufacturerData            = 0x23
authenticate                = 0x2F
cellVoltage4                = 0x3C
cellVoltage3                = 0x3D
cellVoltage2                = 0x3E
cellVoltage1                = 0x3F
BTPDischargeSet             = 0x4A
BTPChargeSet                = 0x4B
stateOfHealth               = 0x4F














