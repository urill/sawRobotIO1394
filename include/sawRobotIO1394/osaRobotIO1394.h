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

#ifndef _osaRobotIO1394_h
#define _osaRobotIO1394_h

#ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstParameterTypes/prmJointType.h>
#else // ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <sawRobotIO1394/EigenWrapper.h>
#include <sawRobotIO1394/MinimalPrm.h>
#endif // ifndef SAW_ROBOT_IO_1394_WO_CISST

#include <stdexcept>

namespace sawRobotIO1394 {

    namespace osaIO1394 {
        const size_t MAX_BOARDS = 16;
        const size_t MAX_AXES = 4;

        //! Exceptions for error handling
        class configuration_error  : public std::runtime_error {
        public: explicit configuration_error(const std::string &what) : std::runtime_error(what) { };
        };

        class safety_error : public std::runtime_error {
        public: explicit safety_error(const std::string &what) : std::runtime_error(what) { };
        };

        class hardware_error : public std::runtime_error {
        public: explicit hardware_error(const std::string &what) : std::runtime_error(what) { };
        };

        //! Defines where the potentiometers are positioned, if any.
        typedef enum {
            POTENTIOMETER_UNDEFINED,
            POTENTIOMETER_ON_ACTUATORS,
            POTENTIOMETER_ON_JOINTS
        } PotLocationType;

        struct DriveConfiguration {
            double EffortToCurrentScale;
            double CurrentToBitsScale;
            double CurrentToBitsOffset;
            double BitsToCurrentScale;
            double BitsToCurrentOffset;
            double ActuatorEffortCommandLimit;
            double ActuatorCurrentCommandLimit;
        };

        struct EncoderConfiguration {
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

        struct PotConfiguration {
            double BitsToVoltageScale;
            double BitsToVoltageOffset;
            double VoltageToPositionScale;
            double VoltageToPositionOffset;
        };

        struct ActuatorConfiguration {
            int BoardID;
            int AxisID;
            prmJointType JointType;

            DriveConfiguration Drive;
            EncoderConfiguration Encoder;
            PotConfiguration Pot;
        };

        struct RobotConfiguration {
            std::string Name;
            int NumberOfActuators;
            int NumberOfJoints;
            bool HasActuatorToJointCoupling;

            osaIO1394::PotLocationType PotLocation;

            std::vector<osaIO1394::ActuatorConfiguration> Actuators;

            vctDoubleMat ActuatorToJointPosition;
            vctDoubleMat JointToActuatorPosition;
            vctDoubleMat ActuatorToJointEffort;
            vctDoubleMat JointToActuatorEffort;
        };

        struct DigitalInputConfiguration {
            std::string Name;
            int BoardID;
            int BitID;
            bool TriggerWhenPressed;
            bool TriggerWhenReleased;
            bool PressedValue;
        };

        struct Configuration {
            std::vector<RobotConfiguration> Robots;
            std::vector<DigitalInputConfiguration> DigitalInputs;
        };
    }
}

#endif // _osaRobotIO1394_h