# MinIMU9ArduinoAHRS
# Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

# Copyright (c) 2011 Pololu Corporation.
# http://www.pololu.com/

# MinIMU9ArduinoAHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
# http://code.google.com/p/sf9domahrs/

# sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
# Julio and Doug Weibel:
# http://code.google.com/p/ardu-imu/

# MinIMU9ArduinoAHRS is free software: you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your option)
# any later version.

# MinIMU9ArduinoAHRS is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
# more details.

# You should have received a copy of the GNU Lesser General Public License along
# with MinIMU9ArduinoAHRS. If not, see <http://www.gnu.org/licenses/>.

##########################################################################

# This is a test/3D visualization program for the Pololu MinIMU-9 + Arduino
# AHRS, based on "Test for Razor 9DOF IMU" by Jose Julio, copyright 2009.

# This script needs VPython, pyserial and pywin modules

# First Install Python 2.6.4 (Python 2.7 also works)
# Install pywin from http://sourceforge.net/projects/pywin32/
# Install pyserial from http://sourceforge.net/projects/pyserial/files/
# Install VPython from http://vpython.org/contents/download_windows.html

from visual import *
import serial
import string
import math
from time import time


grad2rad = 3.141592653 / 180.0

# Main scene
scene = display(title="AHRS")
scene.range = (1.2, 1.2, 1.2)
#scene.forward = (0,-1,-0.25)
scene.forward = (1, 0, -0.25)
scene.up = (0, 0, 1)

# Second scene (Roll, Pitch, Heading)
scene2 = display(title='AHRS', x=0, y=0, width=500,
                 height=200, center=(0, 0, 0), background=(0, 0, 0))
scene2.range = (1, 1, 1)
scene.width = 500
scene.y = 200

scene2.select()
#Roll, Pitch, Heading
cil_roll = cylinder(
    pos=(-0.4, 0, 0), axis=(0.2, 0, 0), radius=0.01, color=color.red)
cil_roll2 = cylinder(
    pos=(-0.4, 0, 0), axis=(-0.2, 0, 0), radius=0.01, color=color.red)
cil_pitch = cylinder(
    pos=(0.1, 0, 0), axis=(0.2, 0, 0), radius=0.01, color=color.green)
cil_pitch2 = cylinder(
    pos=(0.1, 0, 0), axis=(-0.2, 0, 0), radius=0.01, color=color.green)
#cil_course = cylinder(pos=(0.6,0,0),axis=(0.2,0,0),radius=0.01,color=color.blue)
#cil_course2 = cylinder(pos=(0.6,0,0),axis=(-0.2,0,0),radius=0.01,color=color.blue)
arrow_course = arrow(pos=(0.6, 0, 0), color=color.yellow,
                     axis=(-0.2, 0, 0), shaftwidth=0.02, fixedwidth=1)

# Roll,Pitch,Heading labels
label(pos=(-0.4, 0.3, 0), text="Roll", box=0, opacity=0)
label(pos=(0.1, 0.3, 0), text="Pitch", box=0, opacity=0)
label(pos=(0.45, 0.3, 0), text="Heading", box=0, opacity=0)
label(pos=(0.6, 0.22, 0), text="N", box=0, opacity=0, color=color.yellow)
label(pos=(0.6, -0.22, 0), text="S", box=0, opacity=0, color=color.yellow)
label(pos=(0.38, 0, 0), text="W", box=0, opacity=0, color=color.yellow)
label(pos=(0.82, 0, 0), text="E", box=0, opacity=0, color=color.yellow)
label(pos=(0.75, 0.15, 0), height=10, text="NE", box=0, color=color.yellow)
label(pos=(0.45, 0.15, 0), height=10, text="NW", box=0, color=color.yellow)
label(pos=(0.75, -0.15, 0), height=10, text="SE", box=0, color=color.yellow)
label(pos=(0.45, -0.15, 0), height=10, text="SW", box=0, color=color.yellow)

L1 = label(pos=(-0.4, 0.22, 0), text="-", box=0, opacity=0)
L2 = label(pos=(0.1, 0.22, 0), text="-", box=0, opacity=0)
L3 = label(pos=(0.8, 0.3, 0), text="-", box=0, opacity=0)

# Main scene objects
scene.select()
# Reference axis (x,y,z)
arrow(color=color.white, axis=(1, 0, 0), shaftwidth=0.05, fixedwidth=1)
arrow(color=color.white, axis=(0, -1, 0), shaftwidth=0.05, fixedwidth=1)
arrow(color=color.white, axis=(0, 0, -1), shaftwidth=0.05, fixedwidth=1)
# labels
label(pos=(0, 0, 0.8), height=50, text="AHRS",
      box=0, opacity=0, color=color.blue)
label(pos=(1, 0, 0), height=35, text="X", box=0, opacity=0, color=color.green)
label(pos=(0, -1, 0), height=35, text="Y", box=0, opacity=0, color=color.green)
label(pos=(0, 0, -1), height=35, text="Z", box=0, opacity=0, color=color.green)
# IMU object
platform = box(length=1, height=0.05, width=1, color=color.red)
p_line = box(length=1, height=0.08, width=0.1, color=color.red)
plat_arrow = arrow(
    color=color.blue, axis=(1, 0, 0), shaftwidth=0.06, fixedwidth=2)

# Check your COM port and baud rate
# the original serial setting
ser = serial.Serial(port='COM11', baudrate=921600, timeout=1)

# f = open("Serial"+str(time())+".csv", 'w') #   (1)

roll = 0
pitch = 0
Heading = 0

while 1:

    # get the eluer angle from the serial and parser the data
    # the original data format is
    # !%f(roll),%f(pitch),%f(heading)\n
    # and all the float data is singal precision
    line = ser.readline()
    if line.find("!") != -1:          # filter out incomplete (invalid) lines
        line = line.replace("!", "")   # Delete "!"
        print line
        # f.write(line)                     # Write to the output log file(2)
        words = string.split(line, ",")    # Fields split
        if len(words) > 2:
            try:
                roll = float(words[0]) * grad2rad
                pitch = float(words[1]) * grad2rad
                Heading = float(words[2]) * grad2rad
            except:
                print "Invalid line"

            # put the data into the visual sence
            axis = (
                cos(pitch) * cos(Heading), -cos(pitch) * sin(Heading), sin(pitch))
            up = (sin(roll) * sin(Heading) + cos(roll) * sin(pitch) * cos(Heading), sin(roll)
                  * cos(Heading) - cos(roll) * sin(pitch) * sin(Heading), -cos(roll) * cos(pitch))
            platform.axis = axis
            platform.up = up
            platform.length = 1.0
            platform.width = 0.8
            plat_arrow.axis = axis
            plat_arrow.up = up
            plat_arrow.length = 0.9
            p_line.axis = axis
            p_line.up = up
            cil_roll.axis = (0.2 * cos(roll), 0.2 * sin(roll), 0)
            cil_roll2.axis = (-0.2 * cos(roll), -0.2 * sin(roll), 0)
            cil_pitch.axis = (0.2 * cos(pitch), 0.2 * sin(pitch), 0)
            cil_pitch2.axis = (-0.2 * cos(pitch), -0.2 * sin(pitch), 0)
            arrow_course.axis = (0.2 * sin(Heading), 0.2 * cos(Heading), 0)
            L1.text = str(float(words[0]))
            L2.text = str(float(words[1]))
            L3.text = str(float(words[2]))
ser.close
f.close
