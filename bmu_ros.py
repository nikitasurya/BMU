#!/usr/bin/env python

from os.path import dirname
from batt_info.msg import batt_info
import sys

sys.path.append((dirname(dirname(__file))))

import roslib
import rospy

from BMU import BMU
import gpioC2 as gpio

class battManagement(object):
    
    def __init__(self):
        
        #Setup GPIO to output
        gpio.setup(gpio.PIN.PIN22,gpio.DIRECTION.OUT)
        gpio.setup(gpio.PIN.PIN24,gpio.DIRECTION.IN)
        #Kill pin is active low
        gpio.write( 1, gpio.PIN.PIN22)

        #Initializing ROS parameteres
        rospy.init_node('BMU_node', anonymous=True)

        #Get parameters from launch file
        self.bmu_rate = rospy.get_param('~bmu_rate', 10.0)
        self.bmu_topic = rospy.get_param('~bmu_topic', "/bmu")
        
        #Set rate
        self.rate = rospy.Rate(self.bmu_rate)
        #Create publisher 
        self.pub = rospy.Publisher(self.bmu_topic, batt_info, queue_size=10)
        
        #Create instance of BMU message
        self.bmu_msg = batt_info()

        #Create BMU instance
        self.bmu = BMU(0x0B,1)
    

    def _update(self):
        self.bmu_msg.remainingCapacity  = self.bmu.remainingCapacity()
        self.bmu_msg.cellVolt1          = self.bmu.cell_voltage(1)
        self.bmu_msg.cellVolt2          = self.bmu.cell_voltage(2)
        self.bmu_msg.cellVolt3          = self.bmu.cell_voltage(3)
        self.bmu_msg.totalVoltage       = self.bmu.totalCellVoltage()
        self.bmu_msg.timeRemaining      = self.bmu.time_remaining() 
        
        #TODO: Add permanent failures, if fail, do something
        #TODO: remove this after correction
        self.bmu_msg.permanentFailures  = 0.0
        #self.bmu_msg.permanentFailures  = self.bmu.

        #Publish message
        self.pub(self.bmu_msg)

if __name__ == '__main__':
    batt = battManagement()
    while True:
        batt._update()
