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
       
        self.button_flag = False
        self.timenow = 0.0
        #Setup GPIO to output
        #Go to odroid website to get which wiringPi pin to use
        self.pin22 = 6  #interrupt
        self.pin24 = 10 #kill 
        wpi.wiringPiSetup()
        #mentioning pin mode of KILL and INT
        wpi.pinMode(self.pin22,0)
        wpi.pinMode(self.pin24,1)

        #Kill pin is active low
        wpi.digitalWrite(self.pin24,1)

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
        self.bmu.onDCHG()
        #The KILL function is initiated after 5 seconds so that the LTC2954-2 is not getting commands during the lockout time
        time.sleep(3)
        wpi.digitalWrite(self.pin24,0)#Turns off push button by setting kill to low.
        print "Push Button off"
        time.sleep(3)
        wpi.digitalWrite(self.pin24,1)#Sets kill to high, but this does not turn on the push button but prepares it in the event that it is turned on
        

    def _checkState(self):
        #TODO check if current too high
        #TODO check if voltage is too low

        if(self.button_flag and time.time() > self.timenow+5):
            print "Turn off initiated"

        if(wpi.digitalRead(self.pin22) == 0):
            print self.timenow , "True"
            if(self.button_flag == False):
                self.timenow = time.time()
                self.button_flag = True
        else:
            #print "False"
            if(self.button_flag == True):
                #if button was pressed but false shutdown
                print "Push Button Off"
                wpi.digitalWrite(self.pin24,0)
                time.sleep(0.5)
                wpi.digitalWrite(self.pin24,1)
                self.button_flag = False

        #TODO print telementry asking the user to press button again
   
    
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
