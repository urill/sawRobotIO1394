/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Jonathan Bohren
  Created on: 2013-06-29

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaConfiguration1394_h
#define _osaConfiguration1394_h

#ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstParameterTypes/prmJointType.h>

#else // ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <sawRobotIO1394/EigenWrapper.h>
#include <sawRobotIO1394/MinimalPrm.h>
#endif // ifndef SAW_ROBOT_IO_1394_WO_CISST

#include <stdexcept>

class AmpIO;

namespace sawRobotIO1394 {

    const size_t MAX_BOARDS = 16;
    const size_t MAX_AXES = 4;

    //! Exceptions for error handling
    class osaRuntimeError1394: public std::runtime_error {
    public:
        explicit osaRuntimeError1394(const std::string & what):
            std::runtime_error(what) { };
    };


    //! Defines where the potentiometers are positioned, if any.
    typedef enum {
        POTENTIOMETER_UNDEFINED,
        POTENTIOMETER_ON_ACTUATORS,
        POTENTIOMETER_ON_JOINTS
    } osaPot1394Location;

    struct osaDrive1394Configuration {
        double EffortToCurrentScale;
        double CurrentToBitsScale;
        double CurrentToBitsOffset;
        double BitsToCurrentScale;
        double BitsToCurrentOffset;
        double ActuatorEffortCommandLimit;
        double ActuatorCurrentCommandLimit;
    };

    struct osaEncoder1394Configuration {
        double BitsToPositionScale;
        double BitsToPositionOffset;
        double BitsToDPositionScale;
        double BitsToDPositionOffset;
        double BitsToDTimeScale;
        double BitsToDTimeOffset;
        double BitsToVelocityScale;
        double BitsToVelocityOffset;
        int CountsPerTurn;
    };

    struct osaPot1394Configuration {
        double BitsToVoltageScale;
        double BitsToVoltageOffset;
        double VoltageToPositionScale;
        double VoltageToPositionOffset;
    };

    struct osaActuator1394Configuration {
        int BoardID;
        int AxisID;
        prmJointType JointType;

        osaDrive1394Configuration Drive;
        osaEncoder1394Configuration Encoder;
        osaPot1394Configuration Pot;
    };

    struct osaRobot1394Configuration {
        std::string Name;
        int NumberOfActuators;
        int NumberOfJoints;
        bool HasActuatorToJointCoupling;

        osaPot1394Location PotLocation;

        std::vector<osaActuator1394Configuration> Actuators;

        vctDoubleMat ActuatorToJointPosition;
        vctDoubleMat JointToActuatorPosition;
        vctDoubleMat ActuatorToJointEffort;
        vctDoubleMat JointToActuatorEffort;
    };

    struct osaDigitalInput1394Configuration {
        std::string Name;
        int BoardID;
        int BitID;
        bool TriggerWhenPressed;
        bool TriggerWhenReleased;
        bool PressedValue;
    };

    struct osaPort1394Configuration {
        std::vector<osaRobot1394Configuration> Robots;
        std::vector<osaDigitalInput1394Configuration> DigitalInputs;
    };

    // Maps an actuator to the hardware (board and axis)
    struct osaActuatorMapping {
        AmpIO *board;
        int axis;
    };

} // namespace sawRobotIO1394

#endif // _osaConfiguration1394_h
