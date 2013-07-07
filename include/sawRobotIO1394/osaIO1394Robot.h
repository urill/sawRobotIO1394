/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaIO1394Robot_h
#define _osaIO1394Robot_h

#include <vector>
#include <map>
#include <stdint.h>

#ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <cisstCommon/cmnXMLPath.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstParameterTypes/prmJointType.h>
#else
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "EigenWrapper.h"
#include "MinimalPrm.h"
#endif

#include <sawRobotIO1394/osaRobotIO1394.h>

class AmpIO;

namespace sawRobotIO1394 {
    /**
     * IO1394 Robot Abstraction Layer
     * This class is responsible for robot-level actuation and unit
     * conversion as well as safety monitoring for the QLA robot control
     * architecture.
     **/

    class osaIO1394Robot {

    public:
        //! Watchdog counts per ms (note counter width, e.g. 16 bits)
        static const size_t WATCHDOG_MS_TO_COUNT = 192;

        /** \name Lifecycle
         *\{**/
        osaIO1394Robot(const osaIO1394::RobotConfiguration & config,
                       const size_t max_consecutive_current_safety_violations = 100,
                       const size_t actuator_current_buffer_size = 1000);

        void Configure(const osaIO1394::RobotConfiguration & config);

        void SetBoards(std::vector<AmpIO*> boards);
        /**}**/

        /** \name State Update Functions
         * These functions interact with the lower-level hardware to query
         * information only and update this class' members.
         *\{**/
        void PollValidity(void);
        void PollState(void);
        void ConvertState(void);
        void CheckState(void);
        /**}**/

        /** \name Command Functions
         * These functions interact with the lower-level hardware when called to
         * change its state in some way. Note that these functions do not have
         * any side-effects in the class.
         *\{**/
        //
        //! Power / Safety Control
        void EnablePower(void);
        void EnableBoardsPower(void);
        void DisablePower(void);
        void DisableBoardPower(void);
        void SetSafetyRelay(const bool & enabled);
        void SetWatchdogPeriod(const double & periodInSeconds);

        void SetActuatorPower(const bool & enabled);
        void SetActuatorPower(const vctBoolVec & enabled);

        //! Encoder Control
        void SetEncoderPosition(const vctDoubleVec & pos);
        void SetEncoderPositionBits(const vctIntVec & bits);
        void SetSingleEncoderPosition(const int index, const double pos = 0);
        void SetSingleEncoderPositionBits(const int index, const int bits = 0);

        //! Actuator Control
        void SetJointEffort(const vctDoubleVec & efforts);
        void SetActuatorEffort(const vctDoubleVec & efforts);
        void SetActuatorCurrent(const vctDoubleVec & currents);
        void SetActuatorCurrentBits(const vctIntVec & bits);
        /**}**/


        /** \name State Accessors
         * These accessors only access data which is contained in this class, i.e.
         * they do not interact with the lower-level hardware. To update these data
         * from the lower-level system, you must call \ref poll_state.
         *\{**/
        bool Valid(void) const;
        bool PowerStatus(void) const;
        bool SafetyRelay(void) const;
        bool WatchdogStatus(void) const;

#if 0 // defined but not implemented - when implemented, remove equivalent methods from mtsIO1394Robot
        void GetDigitalInput(vctBoolVec & digital) const;

        void get_encoder_pos(vctDoubleVec & pos) const;
        void get_encoder_pos_bits(vctIntVec & bits) const;

        void get_encoder_vel(vctDoubleVec & vel) const;
        void get_encoder_vel_bits(vctIntVec & bits) const;

        void get_actuator_command_efforts(vctDoubleVec & efforts) const;
        void get_actuator_command_amps(vctDoubleVec & currents) const;
        void get_actuator_command_bits(vctIntVec & bits) const;

        void get_actuator_feedback_efforts(vctDoubleVec & efforts) const;
        void get_actuator_feedback_amps(vctDoubleVec & currents) const;
        void get_actuator_feedback_bits(vctIntVec & bits) const;

        void get_actuator_enabled(vctBoolVec & enabled) const;
        void get_actuator_status(vctBoolVec & enabled) const;
#endif

        /**}**/

        /** \name Parameter Accessors
         *\{**/
        osaIO1394::RobotConfiguration GetConfiguration(void) const;
        std::string Name(void) const;
        double NumberOfJoints(void) const;
        double NumberOfActuators(void) const;
        void GetJointTypes(prmJointTypeVec & jointTypes) const;
        void GetJointEffortCommandLimits(vctDoubleVec & limits) const;
        void GetActuatorEffortCommandLimits(vctDoubleVec & limits) const;
        void GetActuatorCurrentCommandLimits(vctDoubleVec & limits) const;
        /**}**/

        /** \name Bias Calibration Functions
         *\{**/
        void CalibrateCurrentCommandOffsetsRequest(const int & numberOfSamples);
        void CalibrateCurrentCommandOffsets(void);
        void CalibrateEncoderOffsetsFromPots(void);

        /**}**/

        /** \name Conversion Functions
         * These functions convert data between units for various purposes. They
         * have no side-effects.
         *\{**/
        //! Conversions for encoders
        void EncoderPositionToBits(const vctDoubleVec & pos, vctIntVec & bits) const;
        void EncoderBitsToPosition(const vctIntVec & bits, vctDoubleVec & pos) const;
        void EncoderBitsToDPosition(const vctIntVec & bits, vctDoubleVec & dpos) const;
        void EncoderBitsToDTime(const vctIntVec & bits, vctDoubleVec & dt) const;
        void EncoderBitsToVelocity(const vctIntVec & bits, vctDoubleVec & vel) const;

        //! Conversions for current commands amd measurements
        void ActuatorEffortToCurrent(const vctDoubleVec & efforts, vctDoubleVec & currents) const;
        void ActuatorCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const;
        void ActuatorBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const;
        void ActuatorCurrentToEffort(const vctDoubleVec & currents, vctDoubleVec & efforts) const;

        //! Conversions for potentiometers
        void PotBitsToVoltage(const vctIntVec & bits, vctDoubleVec & voltages) const;
        void PotVoltageToPosition(const vctDoubleVec & voltages, vctDoubleVec & pos) const;
        /**}**/

    protected:

        void ClipActuatorEffort(vctDoubleVec & efforts);
        void ClipActuatorCurrent(vctDoubleVec & currents);

        //! Board Objects
        std::vector<AmpIO*> Boards_;
        std::map<int, AmpIO*> UniqueBoards_;
        typedef std::map<int, AmpIO*>::iterator unique_board_iterator;
        typedef std::map<int, AmpIO*>::const_iterator unique_board_const_iterator;

        //! Robot Configuration
        osaIO1394::RobotConfiguration Configuration_;
        std::string Name_;
        size_t NumberOfActuators_;
        size_t NumberOfJoints_;

        //! Vectors of actuator properties
        vctIntVec
            BoardAxes_,
            CountsPerTurn_;

        vctDoubleVec
            EffortToCurrentScales_,
            CurrentToBitsScales_,
            CurrentToBitsOffsets_,
            BitsToCurrentScales_,
            BitsToCurrentOffsets_,
            BitsToPositionScales_,
            BitsToPositionOffsets_,
            BitsToDPositionScales_,
            BitsToDPositionOffsets_,
            BitsToDTimeScales_,
            BitsToDTimeOffsets_,
            BitsToVecocityScales_,
            BitsToVelocityOffsets_,
            BitsToVoltageScales_,
            BitsToVoltageOffsets_,
            VoltageToPositionScales_,
            VoltageToPositionOffsets_;

        vctDoubleVec
            JointEffortCommandLimits_,
            ActuatorEffortCommandLimits_,
            ActuatorCurrentCommandLimits_,
            ActuatorCurrentFeedbackLimits_;

        //! Robot type
        prmJointTypeVec JointType_;
        osaIO1394::PotLocationType PotType_;

        //! State Members
        bool
            Valid_,
            PowerStatus_,
            PreviousPowerStatus_,
            WatchdogStatus_;

        unsigned short SafetyRelay_;

        vctBoolVec
            ActuatorPowerStatus_,
            ActuatorPowerEnabled_,
            DigitalInputs_;

        vctIntVec
            PotBits_,
            EncoderPositionBits_,
            EncoderVelocityBits_,
            EncoderDPositionBits_,
            EncoderDTimeBits_;

        vctIntVec
            ActuatorCurrentBitsCommand_,
            ActuatorCurrentBitsFeedback_;

        vctDoubleVec
            PotVoltage_,
            PotPosition_,
            EncoderPosition_,
            EncoderVelocity_,
            EncoderDPosition_,
            EncoderDTime_,
            JointPosition_,
            JointVelocity_,
            ActuatorCurrentCommand_,
            ActuatorEffortCommand_,
            ActuatorCurrentFeedback_,
            ActuatorEffortFeedback_,
            Temperature_;

        //! Actuator current measurement structures
        bool CalibrateCurrentCommandOffsetRequested_;

        size_t
            CurrentSafetyViolationsCounter_,
            CurrentSafetyViolationsMaximum_,
            CalibrateCurrentBufferSize_,
            CalibrateCurrentBufferIndex_;

        //! Buffers used to accumulate current requested and feedback to calibrate requested current offsets
        std::vector<vctDoubleVec>
            CalibrateCurrentCommandBuffers_,
            CalibrateCurrentFeedbackBuffers_;
    };
}

#endif // _osaIO1394Robot_h
