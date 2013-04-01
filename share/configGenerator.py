#!/usr/bin/env python

# python
# 2013-03-31
# Zihan Chen

import numpy as np
import math
import xml.etree.ElementTree as ET

# PSM

# constants 
numOfActuator = 7
numOfJoint = 7

# motor constants Unit: V
motorVol = np.array([24, 24, 24, 24, 24, 24, 24])

# motor default current Unit: A 
motorDefCur = np.array([1.34, 1.34, 0.67, 0.67, 0.67, 0.67, 0.67])

# motor max current Unit: A
motorMaxCur = np.array([2.01, 2.01, 1.005, 1.005, 1.005, 1.005, 1.005])

# motor toruqe const  Unit: Nm/A
motorTor = np.array([0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438])

# Gear ratio 
gearRatio = np.array([56.50, 56.50, 336.6, 11.71, 11.71, 11.71, 11.71])

# Encoder counts per turn 
encCPT = np.array([14400, 14400, 14400, 4000, 4000, 4000, 4000])

# Pitch
# 1 for revolute, mm/deg for prismatic 
pitch = np.array([1, 1, 17.4533, 1, 1, 1, 1])

# compute 
boardID = [8, 9]

# ==== POT =======
# raw value form Intuitive Surgical Inc
# NOTE: 
#    1. 12 bit ADC 
#    2. 0-5 V (Typical)
#    3. Unit: Radian
#    
#    jointAngle = potGain * potADCCnts + potOffset 
# 
jointUpper = np.array([1.5985, 0.9323, 0.24321, 3.0264, 3.0326, 3.0346, 3.0376])
jointLower = np.array([-1.616, -0.97237, -0.0023366, -3.052, -3.0445, -3.0488, -3.0486])
potGain = np.array([-0.00085, -0.00056, 0.0000655, -0.0015, -0.001517, -0.0015, -0.0015])
potOffset = np.array([1.7356, 1.1874, -0.014153, 3.0777, 3.0824, 3.0529, 3.1403])


# =============================================
# XML values 
# =============================================
# === Drive =======
# Direction 
driveDirection = np.array([-1, -1, 1, -1, -1, 1, 1])
AmpsToBitsScale = driveDirection * 5242.8800
AmpsToBitsOffset = np.ones(7) * math.pow(2,15)

BitsToFbAmpsScale = driveDirection * 0.000190738
BitsToFbAmpsOffset = driveDirection * 6.25 * -1.0;

NmToAmps = np.ones(7) / gearRatio / motorTor
MaxCurrent = motorDefCur


# === Encoder ======
BitsToPosSIScale = 360.0 / encCPT * pitch / gearRatio * driveDirection
BitsToPosSIOffset = np.zeros(7)
# % NOT valid for now 
BitsToDeltaPosSI = np.ones(7) * -1
BitsToDeltaT = np.ones(7) * -1
CountsPerTurn = encCPT

# === AnalogIn =====
BitsToVoltsScale = np.ones(7) * 0.0000686656
BitsToVoltsOffset = np.zeros(7)
VoltsToPosSIScale = potGain * math.pow(2,12) / (4.5 - 0.0) * 180.0 / math.pi
VoltsToPosSIOffset = potOffset * 180.0 / math.pi


# ==============================
# Read and Write XML file
# ==============================
filename = '/home/adeguet1/devel/cisst/trunk/saw/applications/sawIntuitiveResearchKit/share/sawRobotIO1394-PSM1.xml'

tree = ET.parse(filename)
config = tree.getroot()

# robot 
robot = config.find('Robot')
robot.set('Name', 'PSM1')
robot.set('NumOfActuator', str(numOfActuator))
robot.set('NumOfJoint', str(numOfJoint))

# Actuator
actuatorID = 0
for Actuator in robot.findall('Actuator'):
    # actuator attrib
    Actuator.set('ActuatorID', str(actuatorID))
    Actuator.set('AxisID', str(actuatorID % 4))
    Actuator.set('BoardID', str( boardID[ actuatorID / 4] ))

    # drive 
    Drive = Actuator.find('Drive')
    Drive.find('AmpsToBits').set('Scale', str(AmpsToBitsScale[actuatorID]))
    Drive.find('AmpsToBits').set('Offset', str(AmpsToBitsOffset[actuatorID]))
    Drive.find('BitsToFeedbackAmps').set('Scale', str(BitsToFbAmpsScale[actuatorID]))
    Drive.find('BitsToFeedbackAmps').set('Offset', str(BitsToFbAmpsOffset[actuatorID]))
    Drive.find('NmToAmps').set('Scale', str(NmToAmps[actuatorID]))
    Drive.find('MaxCurrent').set('Value', str(MaxCurrent[actuatorID]))

    # encoder 
    Encoder = Actuator.find('Encoder')
    Encoder.find('BitsToPosSI').set('Scale', str(BitsToPosSIScale[actuatorID]))
    Encoder.find('BitsToPosSI').set('Offset', str(BitsToPosSIOffset[actuatorID]))
    # TODO: add BitsToDeltaPosSI here
    Encoder.find('CountsPerTurn').set('Value', str(CountsPerTurn[actuatorID]))

    # analogIn
    AnalogIn = Actuator.find('AnalogIn')
    AnalogIn.find('BitsToVolts').set('Scale', str(BitsToVoltsScale[actuatorID]))
    AnalogIn.find('BitsToVolts').set('Offset', str(BitsToVoltsOffset[actuatorID]))
    AnalogIn.find('VoltsToPosSI').set('Scale', str(VoltsToPosSIScale[actuatorID]))
    AnalogIn.find('VoltsToPosSI').set('Offset', str(VoltsToPosSIOffset[actuatorID]))
    
    # increment actuatorID
    actuatorID = actuatorID + 1


# You might want to add coupling matrix here


tree.write(filename)

print 'Done'
