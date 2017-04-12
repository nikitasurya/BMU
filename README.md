## BMU
This project is written for Odroid C2 for the second verion of the buoy project. The battery management is required to monitor the state of health of the battery, status and will cut power if the conditions exceeds the design limitation of the battery. It uses a TIBQ4050 which uses Compensated End of Discharge Voltage (CEDV) to accurately guage the battery.

## Prerequisites
1. Odroid C2 with Ubuntu installed 
2. WiringPi2 for Odroid
3. ROS installed
4. Change Odroid I2C Frequency(400Khz Default) to 100Khz

## Usage of BMU Class
Create an instance of BMU class bmuObject = BMU(deviceAdd, busnum) where deviceAdd is the I2C address of the device and busnum is the SMbus number. The Odroid C2 has 2 SMbus, bus 1 and 2. The default bus used is bus 1.

Most function the important functions have been coded out. To use the function just call bmuObject.toggle_DCHG_FET() to toggle discharge fet or bmuObject.operationStatusReg() to get the operation status register. For more details on what each register means, refer to http://www.ti.com/lit/ug/sluuaq3/sluuaq3.pdf

## BMU ROS
BMU ROS is written to start managing power upon boot. The problem arises because the Odroid requires power to manage the BMU, but the power has to come through the BMU hence the chicken and the egg problem. So a push button circuit was implement to power the Odroid temporary and after that, the Odroid takes over the power management. Upon BMU ROS turns on the discharge fet and turns off the push button circuit. In addition, the push button circuit is then used by the user to tell the system to shutdown.

## Author
1. Louis Goh (Louis_goh@sutd.edu.sg)
2. Nikita (ns283@snu.edu.in)

## License
The MIT License (MIT)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
