import sys
import os

if os.getuid() != 0:
    print "Must have superuser privileges!!!"
    exit(1)

# --------------------------------------------------------------------------------

def setup(pin, direction):
    os.system("echo " + pin + " > /sys/class/gpio/export")
    os.system("echo " + direction + " > /sys/class/gpio/gpio" + pin + "/direction")

def write(data, pin):
    os.system("echo " + str(data) + " > /sys/class/gpio/gpio" + pin + "/value")

def read(pin):
    os.system("cat " + "/sys/class/gpio/gpio" + pin + "/value")
    try:
          with open("/sys/class/gpio/gpio" + pin + "/value") as pin:
                  return pin.read(1)
    except:
          print "Remember to export the pin first!"

class PIN:
    PIN7    = "249"
    GPIO249 = "249"
    PIN11   = "247"
    GPIO247 = "247"
    PIN12   = "238"   
    GPIO238 = "238"
    PIN13   = "239"   
    GPIO239 = "239"
    PIN15   = "237"
    GPIO237 = "237"
    PIN16   = "236"
    GPIO236 = "236"
    PIN18   = "233"
    GPIO233 = "233"
    PIN19   = "235"
    GPIO235 = "235"
    PIN21   = "232"
    GPIO232 = "232"
    PIN22   = "231"
    GPIO231 = "231"
    PIN23   = "230"
    GPIO230 = "230"
    PIN24   = "229"
    GPIO229 = "229"
    PIN26   = "225"
    GPIO225 = "225"
    PIN29   = "228"
    GPIO228 = "228"
    PIN31   = "219"
    GPIO219 = "219"
    PIN32   = "224"
    GPIO224 = "224"
    PIN33   = "234"
    GPIO234 = "234"
    PIN35   = "214"
    GPIO214 = "214"
    PIN36   = "218"
    GPIO218 = "218"

class DIRECTION:
    OUT = "out"
    IN = "in"
