#!/usr/bin/env python

from os.path import dirname
import sys

sys.path.append((dirname(dirname(__file__))))

import roslib
import rospy
from battery.msg import batt_info
from BMU import BMU
import time
import wiringpi2 as wpi

timeDelay = 5.0

class battManagement(object):
    
    def __init__(self):
        
        #Setup GPIO to output
        #Go to odroid website to get which wiringPi pin to use
        self.pin22 = 6  #KILL
        self.pin24 = 10 #INTERRUPT 
        wpi.wiringPiSetup()
        #mentioning pin mode of KILL and INT
        wpi.pinMode(self.pin22,1)
        wpi.pinMode(self.pin24,0)

        #Kill pin is active low
        wpi.digitalWrite(self.pin22,1)

        #Initializing ROS parameteres
        rospy.init_node('BMU_node', anonymous=True)

        #Get parameters from launch file
        self.bmu_rate = rospy.get_param('~bmu_rate', 1.0)
        self.bmu_topic = rospy.get_param('~bmu_topic', "/bmu")
        #self.bmuAdd = rospy.get_param('~bmu_add', "0x0B")
        #Set rate
        self.rate = rospy.Rate(self.bmu_rate)
        #Create publisher 
        self.pub = rospy.Publisher(self.bmu_topic, batt_info, queue_size=10)
        
        #Create instance of BMU message
        self.bmu_msg = batt_info()

        #Create BMU instance
        self.bmu = BMU(0x0B,1)
        self.onDCHG()
        #The KILL function is initiated after 5 seconds so that the LTC2954-2 is not getting commands during the lockout time
        time.sleep(5)
        wpi.digitalWrite(self.pin22,0)
      

    def onDCHG(self):
        """
        To turn ON the DCHG FET 
        Whatever be the previous status of the FET
        """
        flag = 0
        flag = self.bmu.manufacturingStatusReg()
        print bin(flag)
        print format(flag, '#04X')

       
        if((0x04 & flag) == 0):
            print "toggle"
            self.bmu.toggle_DCHG_FET()


    def offDCHG(self):
        """
        To turn OFF the DCHG FET 
        Whatever be the previous status
        """
        flag = self.bmu.operationStatusReg()
        if (0x0002 & flag):
            self.bmu.toggle_DCHG_FET()

        print flag



    def _checkState(self):
        #TODO check if current too high
        #TODO check if voltage is too low
       print "Push the button to turn ON the push button circuit"
       wpi.digitalWrite(self.pin22,1)
       time.sleep(5)
       print "Push Button circuit is ON"
       print "Push and hold for 5 seconds to turn OFF the push button circuit"
       if(wpi.digitalRead(self.pin24)==0):
            #TODO print telementry asking the user to press button again
            print "Turn OFF initiated"
            time.sleep(4)
            if (wpi.digitalRead(self.pin24)==0):
                print "turning OFF"
                wpi.digitalWrite(self.pin22,0)
            
    
    
    def _update(self):
        self.bmu_msg.remainingCapacity  = self.bmu.remainingCapacity()
        self.bmu_msg.cellVolt1          = self.bmu.cell_voltage(1)
        self.bmu_msg.cellVolt2          = self.bmu.cell_voltage(2)
        self.bmu_msg.cellVolt3          = self.bmu.cell_voltage(3)
        self.bmu_msg.totalVoltage       = self.bmu.totalCellVoltage()

        #TODO time remaining returns none type that cause publisher to fail
        #TODO check if when time_remaining returns a proper value the if condition still works
        temp = self.bmu.time_remaining()
        if (temp):
            self.bmu_msg.timeRemaining = self.bmu.time_remaining() 
        else:
            self.bmu_msg.timeRemaining = -1.0
        
        #TODO: Add permanent failures, if fail, do something
        #TODO: remove this after correction
        self.bmu_msg.permanentFailures  = 0.0
        #self.bmu_msg.permanentFailures  = self.bmu.

        #Publish message
        self.pub.publish(self.bmu_msg)
        

if __name__ == '__main__':
    batt = battManagement()
    while True:
        batt._update()
        batt._checkState()
